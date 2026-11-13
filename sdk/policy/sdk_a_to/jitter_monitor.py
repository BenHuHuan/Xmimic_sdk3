"""Real-time high-frequency jitter (chatter) and twitch ("itching") detector.

Evaluates 1000 Hz joint velocity and actuator torque streams to surface
high-frequency oscillations (>12 Hz) and sudden torque/acceleration spikes
that are otherwise only visible in post-hoc numerical logs.
"""

from __future__ import annotations

import math
import numpy as np


class JitterMonitor:
    """Per-joint high-frequency oscillation and impulse/twitch monitor."""

    WARN_THRESHOLD = 0.25
    CRIT_THRESHOLD = 0.55

    def __init__(
        self,
        joint_names: list[str],
        effort_limits: np.ndarray | list[float],
        dt: float = 0.001,
    ) -> None:
        self.joint_names = list(joint_names)
        self.n = len(self.joint_names)
        self.dt = float(dt)
        self.effort_limits = np.maximum(np.asarray(effort_limits, dtype=np.float64), 1.0)

        # High-pass filter states (12 Hz cutoff)
        self._alpha_lp = self.dt / (1.0 / (2.0 * math.pi * 12.0) + self.dt)
        self._alpha_rms = self.dt / (0.15 + self.dt)
        self._rev_decay = math.exp(-self.dt / 0.25)
        self._twitch_decay = math.exp(-self.dt / 0.45)

        self.reset()

    def reset(self) -> None:
        self._initialized = False
        self._tau_lp = np.zeros(self.n, dtype=np.float64)
        self._vel_lp = np.zeros(self.n, dtype=np.float64)
        self._tau_hp_ms = np.zeros(self.n, dtype=np.float64)
        self._vel_hp_ms = np.zeros(self.n, dtype=np.float64)
        self._prev_tau = np.zeros(self.n, dtype=np.float64)
        self._prev_vel = np.zeros(self.n, dtype=np.float64)
        self._last_sign = np.zeros(self.n, dtype=np.int8)
        self._rev_rate = np.zeros(self.n, dtype=np.float64)
        self._twitch_env = np.zeros(self.n, dtype=np.float64)

        self.severity = np.zeros(self.n, dtype=np.float64)
        self.jitter = np.zeros(self.n, dtype=np.float64)
        self.freq_hz = np.zeros(self.n, dtype=np.float64)
        self.jitter_score = 0.0
        self.twitch_score = 0.0
        self.peak_joint_idx = 0
        self.peak_freq_hz = 0.0
        self.active_joints = 0

        # Episode summary accumulators
        self.steps = 0
        self.warn_steps = 0
        self.crit_steps = 0
        self.max_jitter_score = 0.0
        self.max_twitch_score = 0.0
        self._joint_max_severity = np.zeros(self.n, dtype=np.float64)
        self._joint_warn_steps = np.zeros(self.n, dtype=np.int64)

    def update(self, qvel: np.ndarray, torque: np.ndarray) -> None:
        vel = np.asarray(qvel, dtype=np.float64)
        tau = np.asarray(torque, dtype=np.float64)
        if not self._initialized:
            self._tau_lp[:] = tau
            self._vel_lp[:] = vel
            self._prev_tau[:] = tau
            self._prev_vel[:] = vel
            self._initialized = True
            return

        # 1. High-pass torque & velocity chatter (>12 Hz)
        self._tau_lp += self._alpha_lp * (tau - self._tau_lp)
        self._vel_lp += self._alpha_lp * (vel - self._vel_lp)
        tau_hp = (tau - self._tau_lp) / self.effort_limits
        vel_hp = vel - self._vel_lp

        self._tau_hp_ms += self._alpha_rms * (tau_hp * tau_hp - self._tau_hp_ms)
        self._vel_hp_ms += self._alpha_rms * (vel_hp * vel_hp - self._vel_hp_ms)
        tau_hp_rms = np.sqrt(np.maximum(self._tau_hp_ms, 0.0))
        vel_hp_rms = np.sqrt(np.maximum(self._vel_hp_ms, 0.0))

        # 2. Velocity sign-reversal frequency (with 0.15 rad/s deadband)
        sign_now = np.zeros(self.n, dtype=np.int8)
        sign_now[vel > 0.15] = 1
        sign_now[vel < -0.15] = -1
        reversals = (sign_now != 0) & (self._last_sign != 0) & (sign_now != self._last_sign)
        nonzero = sign_now != 0
        self._last_sign[nonzero] = sign_now[nonzero]

        self._rev_rate = self._rev_rate * self._rev_decay + reversals.astype(np.float64) / 0.25
        self.freq_hz = 0.5 * self._rev_rate

        # 3. Instantaneous twitch / spike ("itching") envelope
        d_tau = np.abs(tau - self._prev_tau) / self.effort_limits
        d_vel = np.abs(vel - self._prev_vel) / (max(self.dt, 1e-6) * 150.0)
        inst_twitch = np.maximum(d_tau / 0.12, d_vel)
        # Ignore tiny resting numerical noise below 0.08
        inst_twitch = np.where(inst_twitch < 0.08, 0.0, inst_twitch)
        self._twitch_env = np.maximum(inst_twitch, self._twitch_env * self._twitch_decay)

        self._prev_tau[:] = tau
        self._prev_vel[:] = vel

        # 4. Combined per-joint severity
        freq_gain = 0.4 + 0.6 * np.clip(self.freq_hz / 15.0, 0.0, 1.5)
        tau_jitter = np.clip((tau_hp_rms / 0.12) * freq_gain, 0.0, 1.5)
        vel_jitter = np.clip((vel_hp_rms / 1.5) * np.clip(self.freq_hz / 15.0, 0.0, 1.2), 0.0, 1.5)
        self.jitter = np.maximum(tau_jitter, vel_jitter)
        self.severity = np.clip(np.maximum(self.jitter, self._twitch_env), 0.0, 1.5)

        self.jitter_score = float(self.jitter.max())
        self.twitch_score = float(self._twitch_env.max())
        self.peak_joint_idx = int(np.argmax(self.severity))
        self.peak_freq_hz = float(self.freq_hz[self.peak_joint_idx])
        self.active_joints = int((self.severity >= self.WARN_THRESHOLD).sum())

        self.steps += 1
        peak_sev = float(self.severity[self.peak_joint_idx])
        if peak_sev >= self.WARN_THRESHOLD:
            self.warn_steps += 1
        if peak_sev >= self.CRIT_THRESHOLD:
            self.crit_steps += 1
        self.max_jitter_score = max(self.max_jitter_score, self.jitter_score)
        self.max_twitch_score = max(self.max_twitch_score, self.twitch_score)
        self._joint_max_severity = np.maximum(self._joint_max_severity, self.severity)
        self._joint_warn_steps += (self.severity >= self.WARN_THRESHOLD).astype(np.int64)

    def summary(self) -> dict[str, object]:
        order = np.argsort(-self._joint_max_severity)
        top_joints = [
            {
                "joint": self.joint_names[int(idx)],
                "max_severity": round(float(self._joint_max_severity[int(idx)]), 4),
                "warn_fraction": round(float(self._joint_warn_steps[int(idx)]) / max(self.steps, 1), 4),
            }
            for idx in order[:5]
            if self._joint_max_severity[int(idx)] >= 0.05
        ]
        return {
            "max_jitter_score": round(self.max_jitter_score, 4),
            "max_twitch_score": round(self.max_twitch_score, 4),
            "warn_fraction": round(self.warn_steps / max(self.steps, 1), 4),
            "crit_fraction": round(self.crit_steps / max(self.steps, 1), 4),
            "top_joints": top_joints,
        }
