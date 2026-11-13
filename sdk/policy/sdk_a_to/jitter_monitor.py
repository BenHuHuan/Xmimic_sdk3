from __future__ import annotations

import math
import numpy as np


def _butter2_hp_coeffs(fc: float, fs: float) -> tuple[float, float, float, float, float]:
    """2nd-order Butterworth high-pass biquad coefficients (Direct Form II)."""
    w0 = math.tan(math.pi * min(max(fc, 0.1), 0.45 * fs) / fs)
    w02 = w0 * w0
    r2 = math.sqrt(2.0) * w0
    norm = 1.0 + r2 + w02
    b0 = 1.0 / norm
    b1 = -2.0 * b0
    b2 = b0
    a1 = 2.0 * (w02 - 1.0) / norm
    a2 = (1.0 - r2 + w02) / norm
    return b0, b1, b2, a1, a2


class JitterMonitor:
    """Fatal hardware-damage jitter & shock monitor (50 Hz ZOH-immune, full-band).

    Grounded in robotics sim-to-real & electromechanical damage criteria:
      1. Sustained HF Torque Chatter (I²t thermal & gearbox HCF fatigue):
         - 10–25 Hz band: 2nd-order Butterworth HP (>=10 Hz) on 20 ms boxcar-
           averaged torque (zero false positives on 1–4 Hz gaits & 50 Hz ZOH)
           gated by sustained oscillatory reversals.
         - 30–250 Hz band: Intra-window non-monotonic peak-to-peak ripple
           computed after the 2 ms ZOH transient settles (closes the 50/100 Hz
           boxcar blind spot for local PD / contact hunting).
      2. ISO 20816-inspired HF Mechanical Velocity Vibration (>=10 Hz):
         - 2nd-order Butterworth HP (>=10 Hz) RMS velocity + zero-crossing freq,
           with high deadband (>=0.75 rad/s) so micro-buzz is ignored.
      3. Severe Torque-Reversal Shock & kp-Scaled Command Jerk:
         - Actual 20 ms torque 2nd-difference shock (>45% effort_limit), plus
           command jerk converted to torque impact (kp * |Δ² q_des| / effort_limit)
           so stiff joints (waist kp=400) and soft joints (ankle kp=16.8) are
           evaluated on equal physical footing.
    """

    WARN_THRESHOLD = 0.55
    CRIT_THRESHOLD = 0.75

    def __init__(
        self,
        joint_names: list[str],
        effort_limits: np.ndarray | list[float],
        dt: float = 0.001,
        policy_dt: float = 0.02,
        kp: np.ndarray | list[float] | None = None,
    ) -> None:
        self.joint_names = list(joint_names)
        self.n = len(self.joint_names)
        self.dt = float(dt)
        self.policy_dt = float(policy_dt)
        self.substeps = max(1, int(round(self.policy_dt / max(self.dt, 1e-6))))

        limits = np.asarray(effort_limits, dtype=np.float64)
        self.effort_limits = np.maximum(limits, 1.0)
        self.kp = np.asarray(kp, dtype=np.float64) if kp is not None else None

        fs_fast = 1.0 / max(self.dt, 1e-6)
        eff_policy_dt = self.substeps * self.dt
        fs_policy = 1.0 / max(eff_policy_dt, 1e-6)

        # 2nd-order 10 Hz Butterworth high-pass filters (rejects <=4 Hz locomotion by >25x)
        self._vel_bq = _butter2_hp_coeffs(10.0, fs_fast)
        self._tau_bq = _butter2_hp_coeffs(10.0, fs_policy)

        # Envelopes & persistence (0.12 s attack rejects single-frame transients)
        self._alpha_vel_rms = math.exp(-self.dt / 0.15)
        self._alpha_zc = math.exp(-self.dt / 0.20)
        self._alpha_jitter_up = math.exp(-self.dt / 0.12)
        self._alpha_jitter_dn = math.exp(-self.dt / 0.18)

        self._alpha_tau_rms = math.exp(-eff_policy_dt / 0.15)
        self._alpha_rev = math.exp(-eff_policy_dt / 0.15)
        self._twitch_decay = math.exp(-eff_policy_dt / 0.18)

        self.reset()

    def reset(self) -> None:
        n = self.n
        self._vel_w1 = np.zeros(n, dtype=np.float64)
        self._vel_w2 = np.zeros(n, dtype=np.float64)
        self._vel_hp_prev = np.zeros(n, dtype=np.float64)
        self._vel_hp_sq = np.zeros(n, dtype=np.float64)
        self._zc_rate = np.zeros(n, dtype=np.float64)

        self._tau_accum = np.zeros(n, dtype=np.float64)
        self._substep_count = 0
        self._intra_min = np.full(n, np.inf, dtype=np.float64)
        self._intra_max = np.full(n, -np.inf, dtype=np.float64)
        self._intra_start = np.zeros(n, dtype=np.float64)
        self._intra_has = False

        self._tau_hist_len = 0
        self._prev_tau_bar = np.zeros(n, dtype=np.float64)
        self._prev_dtau_bar = np.zeros(n, dtype=np.float64)
        self._tau_w1 = np.zeros(n, dtype=np.float64)
        self._tau_w2 = np.zeros(n, dtype=np.float64)
        self._tau_hp_sq = np.zeros(n, dtype=np.float64)
        self._tau_intra_sq = np.zeros(n, dtype=np.float64)
        self._rev_duty = np.zeros(n, dtype=np.float64)
        self._tau_jitter = np.zeros(n, dtype=np.float64)

        self._qdes_hist_len = 0
        self._prev_qdes = np.zeros(n, dtype=np.float64)
        self._prev_dqdes = np.zeros(n, dtype=np.float64)

        self.jitter = np.zeros(n, dtype=np.float64)
        self.twitch = np.zeros(n, dtype=np.float64)
        self.severity = np.zeros(n, dtype=np.float64)
        self.freq_hz = np.zeros(n, dtype=np.float64)

        self.jitter_score = 0.0
        self.twitch_score = 0.0
        self.peak_joint_idx = 0
        self.peak_freq_hz = 0.0
        self.active_joints = 0

        self._steps = 0
        self._warn_steps = 0
        self._crit_steps = 0
        self._max_jitter = 0.0
        self._max_twitch = 0.0
        self._per_joint_max = np.zeros(n, dtype=np.float64)
        self._per_joint_warn_steps = np.zeros(n, dtype=np.int64)

    def _biquad_step(
        self,
        x: np.ndarray,
        w1: np.ndarray,
        w2: np.ndarray,
        coeffs: tuple[float, float, float, float, float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        b0, b1, b2, a1, a2 = coeffs
        w0 = x - a1 * w1 - a2 * w2
        y = b0 * w0 + b1 * w1 + b2 * w2
        return y, w0, w1

    def update(
        self,
        qvel: np.ndarray,
        torque: np.ndarray,
        q_des: np.ndarray | None = None,
    ) -> None:
        qv = np.asarray(qvel, dtype=np.float64)
        tau = np.asarray(torque, dtype=np.float64)

        # 1. ISO 20816-inspired HF velocity vibration (>=10 Hz, 2nd-order Butterworth HP)
        vel_hp, self._vel_w1, self._vel_w2 = self._biquad_step(qv, self._vel_w1, self._vel_w2, self._vel_bq)
        self._vel_hp_sq = self._alpha_vel_rms * self._vel_hp_sq + (1.0 - self._alpha_vel_rms) * (vel_hp * vel_hp)
        vel_hp_rms = np.sqrt(np.maximum(self._vel_hp_sq, 0.0))

        # Ignore tiny numerical/encoder noise below 0.25 rad/s
        noise_gate = vel_hp_rms > 0.25
        crossed = ((vel_hp * self._vel_hp_prev) < 0.0) & noise_gate
        self._zc_rate = self._alpha_zc * self._zc_rate + (1.0 - self._alpha_zc) * (crossed.astype(np.float64) / self.dt)
        self._vel_hp_prev = vel_hp
        self.freq_hz = np.where(noise_gate, 0.5 * self._zc_rate, 0.0)

        # Fatal velocity resonance: >= 10 Hz and >= 0.75 rad/s RMS (2.0 rad/s = 1.0)
        amp_norm = np.clip((vel_hp_rms - 0.75) / 1.25, 0.0, 1.0)
        freq_gate = np.clip((self.freq_hz - 10.0) / 8.0, 0.0, 1.0)
        vel_jitter = np.where(self.freq_hz >= 10.0, amp_norm * (0.5 + 0.5 * freq_gate), 0.0)

        # Track intra-window (30-250 Hz) non-monotonic torque ripple after 2 ms ZOH transient
        self._tau_accum += tau
        if self._substep_count >= 2:
            if not self._intra_has:
                self._intra_start = tau.copy()
                self._intra_min = tau.copy()
                self._intra_max = tau.copy()
                self._intra_has = True
            else:
                self._intra_min = np.minimum(self._intra_min, tau)
                self._intra_max = np.maximum(self._intra_max, tau)
        self._substep_count += 1

        if self._substep_count >= self.substeps:
            tau_bar = self._tau_accum / float(self._substep_count)
            self._tau_accum.fill(0.0)
            self._substep_count = 0

            # Band B (30-250 Hz): intra-window excess oscillation amplitude (monotonic decay -> 0)
            if self._intra_has:
                excess_pp = np.maximum(0.0, (self._intra_max - self._intra_min) - np.abs(tau - self._intra_start))
                intra_rms_norm = (excess_pp / (2.0 * math.sqrt(2.0))) / self.effort_limits
                self._intra_has = False
            else:
                intra_rms_norm = np.zeros(self.n, dtype=np.float64)
            self._tau_intra_sq = self._alpha_tau_rms * self._tau_intra_sq + (1.0 - self._alpha_tau_rms) * (intra_rms_norm * intra_rms_norm)
            intra_rms = np.sqrt(np.maximum(self._tau_intra_sq, 0.0))
            intra_jitter = np.clip((intra_rms - 0.12) / 0.14, 0.0, 1.0)

            # Band A (10-25 Hz): 2nd-order Butterworth HP on 20 ms boxcar torque + reversal gate
            tau_hp, self._tau_w1, self._tau_w2 = self._biquad_step(tau_bar, self._tau_w1, self._tau_w2, self._tau_bq)
            tau_hp_norm = tau_hp / self.effort_limits
            self._tau_hp_sq = self._alpha_tau_rms * self._tau_hp_sq + (1.0 - self._alpha_tau_rms) * (tau_hp_norm * tau_hp_norm)
            tau_hp_rms = np.sqrt(np.maximum(self._tau_hp_sq, 0.0))

            if self._tau_hist_len == 0:
                self._prev_tau_bar = tau_bar.copy()
                self._tau_hist_len = 1
                tau_shock = np.zeros(self.n, dtype=np.float64)
            elif self._tau_hist_len == 1:
                self._prev_dtau_bar = tau_bar - self._prev_tau_bar
                self._prev_tau_bar = tau_bar.copy()
                self._tau_hist_len = 2
                tau_shock = np.zeros(self.n, dtype=np.float64)
            else:
                dtau = tau_bar - self._prev_tau_bar
                dd_tau = np.abs(dtau - self._prev_dtau_bar) / self.effort_limits
                # Oscillatory reversal check (single impact/ramp doesn't sustain rev_duty)
                reversed_dir = ((dtau * self._prev_dtau_bar) < 0.0) & (np.abs(dtau) > 0.05 * self.effort_limits)
                self._rev_duty = self._alpha_rev * self._rev_duty + (1.0 - self._alpha_rev) * reversed_dir.astype(np.float64)
                self._prev_dtau_bar = dtau
                self._prev_tau_bar = tau_bar.copy()

                # Sustained HF torque fatigue/thermal chatter: >11% effort RMS & oscillatory
                rev_gate = np.clip((self._rev_duty - 0.25) / 0.25, 0.0, 1.0)
                band_a_jitter = rev_gate * np.clip((tau_hp_rms - 0.11) / 0.11, 0.0, 1.0)
                self._tau_jitter = np.maximum(band_a_jitter, intra_jitter)

                # Severe torque reversal / impact shock (>45% effort jump in 20 ms)
                tau_shock = np.clip((dd_tau - 0.45) / 0.40, 0.0, 1.0)

            # kp-weighted commanded torque jerk (so waist kp=400 vs ankle kp=16.8 are physically scaled)
            cmd_shock = np.zeros(self.n, dtype=np.float64)
            if q_des is not None:
                qd = np.asarray(q_des, dtype=np.float64)
                if self._qdes_hist_len == 0:
                    self._prev_qdes = qd.copy()
                    self._qdes_hist_len = 1
                elif self._qdes_hist_len == 1:
                    self._prev_dqdes = qd - self._prev_qdes
                    self._prev_qdes = qd.copy()
                    self._qdes_hist_len = 2
                else:
                    dq = qd - self._prev_qdes
                    ddq = np.abs(dq - self._prev_dqdes)
                    self._prev_dqdes = dq
                    self._prev_qdes = qd.copy()
                    if self.kp is not None:
                        cmd_torque_shock = (self.kp * ddq) / self.effort_limits
                        cmd_shock = np.clip((cmd_torque_shock - 0.55) / 0.45, 0.0, 1.0)
                    else:
                        cmd_shock = np.clip((ddq - 0.35) / 0.35, 0.0, 1.0)

            inst_twitch = np.maximum(cmd_shock, tau_shock)
            self.twitch = np.maximum(self.twitch * self._twitch_decay, inst_twitch)

        # 3. Persistence-gated chatter combination
        inst_jitter = np.maximum(vel_jitter, self._tau_jitter)
        rising = inst_jitter > self.jitter
        alpha_j = np.where(rising, self._alpha_jitter_up, self._alpha_jitter_dn)
        self.jitter = alpha_j * self.jitter + (1.0 - alpha_j) * inst_jitter

        self.severity = np.maximum(self.jitter, self.twitch)
        self.jitter_score = float(np.max(self.jitter)) if self.n else 0.0
        self.twitch_score = float(np.max(self.twitch)) if self.n else 0.0
        if self.n:
            self.peak_joint_idx = int(np.argmax(self.severity))
            self.peak_freq_hz = float(self.freq_hz[self.peak_joint_idx])
            self.active_joints = int(np.sum(self.severity >= self.CRIT_THRESHOLD))

        # 4. Episode stats
        self._steps += 1
        peak_sev = max(self.jitter_score, self.twitch_score)
        if peak_sev >= self.WARN_THRESHOLD:
            self._warn_steps += 1
        if peak_sev >= self.CRIT_THRESHOLD:
            self._crit_steps += 1
        if self.jitter_score > self._max_jitter:
            self._max_jitter = self.jitter_score
        if self.twitch_score > self._max_twitch:
            self._max_twitch = self.twitch_score
        self._per_joint_max = np.maximum(self._per_joint_max, self.severity)
        self._per_joint_warn_steps += (self.severity >= self.CRIT_THRESHOLD).astype(np.int64)

    def summary(self) -> dict:
        steps = max(self._steps, 1)
        top_idx = np.argsort(self._per_joint_max)[::-1][:5]
        top_joints = [
            {
                "joint": self.joint_names[i],
                "max_severity": round(float(self._per_joint_max[i]), 3),
                "crit_ratio": round(float(self._per_joint_warn_steps[i]) / steps, 4),
            }
            for i in top_idx
            if self._per_joint_max[i] >= 0.15
        ]
        return {
            "max_jitter": round(self._max_jitter, 3),
            "max_twitch": round(self._max_twitch, 3),
            "warn_ratio": round(self._warn_steps / steps, 4),
            "crit_ratio": round(self._crit_steps / steps, 4),
            "top_joints": top_joints,
        }
