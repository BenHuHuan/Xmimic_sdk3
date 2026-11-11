"""Simulation-aligned PD torque-limit protection for Xrobot3 hardware & sim.

Ensures that position targets (and parallel-ankle sptlib commands) sent to motor
drivers never generate onboard PD torques exceeding the simulation actuator
effort limits (`deploy.json['effort']` / `sdk_a_scene.xml` `ctrlrange`).
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Tuple

import numpy as np


DEFAULT_CONTRACT_PATH = (
    Path(__file__).resolve().parents[1] / "policy" / "sdk_a_to" / "deploy.json"
)


def load_sim_effort_limits(
    contract_path: str | Path | None = None,
    explicit_limits: list[float] | np.ndarray | None = None,
    scale: float = 1.0,
    expected_dof: int = 29,
) -> np.ndarray:
    """Load 29-DOF simulation effort limits (Nm) and apply positive scale."""
    scale = float(scale)
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"torque_limit_scale must be finite and > 0, got {scale}")

    if explicit_limits is not None:
        effort = np.asarray(explicit_limits, dtype=np.float64)
    else:
        path = Path(contract_path) if contract_path else DEFAULT_CONTRACT_PATH
        contract = json.loads(path.read_text())
        effort = np.asarray(contract["effort"], dtype=np.float64)

    if effort.shape != (expected_dof,) or not np.isfinite(effort).all() or np.any(effort <= 0.0):
        raise ValueError(
            f"Invalid simulation effort limits: shape={effort.shape}, values={effort}"
        )
    return effort * scale


def clamp_pd_target_by_torque(
    q_d: np.ndarray,
    q_a: np.ndarray,
    q_dot_a: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    effort_limit: np.ndarray,
    q_dot_d: np.ndarray | None = None,
    tau_d: np.ndarray | None = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Bound `(q_d, tau_d)` so `kp*(q_d - q_a) + kd*(q_dot_d - q_dot_a) + tau_d` stays in `[-effort_limit, +effort_limit]`.

    Unsaturated joints (`|tau_raw| <= effort_limit`) preserve `q_d` and `tau_d`
    bit-for-bit without floating-point perturbation.

    Returns:
        `(q_d_safe, tau_d_safe, tau_raw, tau_clip, clipped_mask)`
    """
    q_d_arr = np.asarray(q_d, dtype=np.float64)
    q_a_arr = np.asarray(q_a, dtype=np.float64)
    dq_a_arr = np.asarray(q_dot_a, dtype=np.float64)
    kp_arr = np.asarray(kp, dtype=np.float64)
    kd_arr = np.asarray(kd, dtype=np.float64)
    lim_arr = np.asarray(effort_limit, dtype=np.float64)
    dq_d_arr = np.zeros_like(q_d_arr) if q_dot_d is None else np.asarray(q_dot_d, dtype=np.float64)
    tau_d_arr = np.zeros_like(q_d_arr) if tau_d is None else np.asarray(tau_d, dtype=np.float64)

    n = q_d_arr.shape[0]
    for arr, name in (
        (q_d_arr, "q_d"),
        (q_a_arr, "q_a"),
        (dq_a_arr, "q_dot_a"),
        (kp_arr, "kp"),
        (kd_arr, "kd"),
        (lim_arr, "effort_limit"),
        (dq_d_arr, "q_dot_d"),
        (tau_d_arr, "tau_d"),
    ):
        if arr.shape != (n,) or not np.isfinite(arr).all():
            raise FloatingPointError(f"Non-finite or mismatched shape in torque limiter input '{name}'")
    if np.any(lim_arr <= 0.0) or np.any(kp_arr < 0.0) or np.any(kd_arr < 0.0):
        raise ValueError("Invalid negative gains or non-positive effort_limit")

    damping_term = kd_arr * (dq_d_arr - dq_a_arr)
    tau_raw = kp_arr * (q_d_arr - q_a_arr) + damping_term + tau_d_arr
    clipped_mask = np.abs(tau_raw) > lim_arr

    q_d_safe = q_d_arr.copy()
    tau_d_safe = tau_d_arr.copy()
    tau_clip = np.clip(tau_raw, -lim_arr, lim_arr)

    if np.any(clipped_mask):
        pos_ctrl = clipped_mask & (kp_arr > 1e-8)
        if np.any(pos_ctrl):
            # Use a tiny sub-mNm safety margin (1 - 1e-6) on saturated joints so
            # downstream float32 wire serialization never rounds 1 ULP over effort_limit.
            safe_tau = np.clip(tau_raw[pos_ctrl], -lim_arr[pos_ctrl] * (1.0 - 1e-6), lim_arr[pos_ctrl] * (1.0 - 1e-6))
            q_d_safe[pos_ctrl] = (
                q_a_arr[pos_ctrl]
                + (safe_tau - damping_term[pos_ctrl] - tau_d_arr[pos_ctrl]) / kp_arr[pos_ctrl]
            )
            tau_clip[pos_ctrl] = safe_tau

        tor_ctrl = clipped_mask & (kp_arr <= 1e-8)
        if np.any(tor_ctrl):
            safe_tau = np.clip(tau_raw[tor_ctrl], -lim_arr[tor_ctrl] * (1.0 - 1e-6), lim_arr[tor_ctrl] * (1.0 - 1e-6))
            tau_d_safe[tor_ctrl] = safe_tau - damping_term[tor_ctrl]
            tau_clip[tor_ctrl] = safe_tau

    return q_d_safe, tau_d_safe, tau_raw, tau_clip, clipped_mask


def clamp_parallel_ankle_torque(
    tor_d_p: np.ndarray,
    effort_limit_4: np.ndarray,
) -> Tuple[np.ndarray, bool]:
    """Direction-preserving torque bound for the 4 parallel ankle motors `[L1, L2, R1, R2]`.

    Scales each ankle's `(m1, m2)` motor pair proportionally when either actuator
    exceeds `effort_limit_4`, preserving the serial `(pitch, roll)` wrench ratio.
    """
    out = np.asarray(tor_d_p, dtype=np.float64).copy()
    lim = np.asarray(effort_limit_4, dtype=np.float64)
    if out.shape != (4,) or lim.shape != (4,) or not np.isfinite(out).all() or np.any(lim <= 0.0):
        raise ValueError("Invalid parallel ankle torque vector or effort limit")

    clipped = False
    for pair in (slice(0, 2), slice(2, 4)):
        ratios = np.abs(out[pair]) / lim[pair]
        peak = float(np.max(ratios))
        if peak > 1.0:
            out[pair] *= (1.0 - 1e-6) / peak
            clipped = True
    return out, clipped


def enforce_wire_float32_torque_limit(
    q_d_motor: np.ndarray,
    q_dot_d_motor: np.ndarray,
    tau_d_motor: np.ndarray,
    q_a_p: np.ndarray,
    q_dot_a_p: np.ndarray,
    kp_p: np.ndarray,
    kd_p: np.ndarray,
    zero_offset: np.ndarray,
    zero_cnt: np.ndarray,
    motor_dir: np.ndarray,
    zero_pos: np.ndarray,
    effort_limit: np.ndarray,
) -> None:
    """In-place guard ensuring IEEE-754 `float32` `MotorCtrl` commands respect `effort_limit`."""
    pos_a_motor = ((q_a_p - zero_offset - 2.0 * np.pi * zero_cnt) * motor_dir + zero_pos).astype(np.float32)
    spd_a_motor = (q_dot_a_p * motor_dir).astype(np.float32)

    pos_d_f32 = q_d_motor.astype(np.float32)
    spd_d_f32 = q_dot_d_motor.astype(np.float32)
    tor_d_f32 = tau_d_motor.astype(np.float32)
    kp_f32 = kp_p.astype(np.float32)
    kd_f32 = kd_p.astype(np.float32)
    lim_f64 = np.asarray(effort_limit, dtype=np.float64)

    tau_wire = (
        kp_f32.astype(np.float64) * (pos_d_f32.astype(np.float64) - pos_a_motor.astype(np.float64))
        + kd_f32.astype(np.float64) * (spd_d_f32.astype(np.float64) - spd_a_motor.astype(np.float64))
        + tor_d_f32.astype(np.float64)
    )
    over = np.flatnonzero(np.abs(tau_wire) > lim_f64)
    for idx in over:
        sign = 1.0 if tau_wire[idx] > 0.0 else -1.0
        target_tau = sign * lim_f64[idx] * (1.0 - 1e-6)
        damp = float(kd_f32[idx]) * (float(spd_d_f32[idx]) - float(spd_a_motor[idx]))
        if kp_f32[idx] > 1e-8:
            p_new = np.float32(float(pos_a_motor[idx]) + (target_tau - damp - float(tor_d_f32[idx])) / float(kp_f32[idx]))
            # Step 1 ULP toward pos_a_motor if float32 rounding still slightly exceeds lim
            for _ in range(4):
                t_chk = float(kp_f32[idx]) * (float(p_new) - float(pos_a_motor[idx])) + damp + float(tor_d_f32[idx])
                if abs(t_chk) <= lim_f64[idx]:
                    break
                p_new = np.nextafter(p_new, pos_a_motor[idx])
            q_d_motor[idx] = float(p_new)
        else:
            t_new = np.float32(target_tau - damp)
            for _ in range(4):
                if abs(damp + float(t_new)) <= lim_f64[idx]:
                    break
                t_new = np.nextafter(t_new, np.float32(0.0))
            tau_d_motor[idx] = float(t_new)
