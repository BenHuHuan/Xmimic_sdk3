"""4-component closed-loop runtime for Tiangong 3 (`SDK-A` + `B_PLUS` + `SafeFall` + `GroundBridge`).

Matches the G1 `State_Stable` + `BPlusPolicy` + `DeployedAutonomousBridge` + `VoluntaryOwner`
architecture tick-for-tick at 50 Hz:
  1. `SDK-A` (`TO17+1`) proposes `c_target` (`29D` rad) and `sdk_raw` (`29D` raw action),
     and produces the updated 4-frame history `sdk_history` (`[1, 884]`).
  2. `B_PLUS` (`bplus_step.onnx`) observes `[q(29), dq(29), quat_wxyz(4), gyro_b(3), acc_b(3)]`
     and outputs `(bplus_target, next_alpha, probability, ...)`.
     - When `alpha > 0`: `B_PLUS + SafeFall` owns the command, immediately preempting any
       active/fading `GroundBridge` option, and synchronizes `SDK-A`'s `previous_action`
       to `(bplus_target - offset) / scale`.
  3. On the protection release falling edge (`prev_alpha > 0` and `alpha == 0`):
     `AutonomousGroundBridge` (`autonomous_ground_bridge.onnx`) resets its `122D` dual-rate
     causal EMA memory to zero and enters `AutonomousBridgeClock` (`VoluntaryOwner` semantics).
  4. While `AutonomousBridgeClock` owns the command (`active` or `5`-tick `100 ms` fade):
     `GroundBridge` consumes `proprio_history = sdk_history.reshape(1, 4, 221)[:, :, 128:]`
     plus `sdk_action = sdk_raw`, checks voluntary release every `decision_steps` (`5` ticks),
     blends smoothly back to live `SDK-A` over `transition_steps` (`5` ticks), and updates
     BOTH `SDK-A.previous_action` and `B_PLUS.previous_target` to the executed command.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort
import yaml


def clamp_tiangong3_parallel_ankles(
    target_rad: np.ndarray,
    *,
    pitch_range: tuple[float, float] = (-1.22, 0.52),
    roll_range: tuple[float, float] = (-0.52, 0.52),
    coupled_envelope: bool = True,
) -> np.ndarray:
    """Optional conservative serial-space clamp for Tiangong 3 parallel ankle linkages."""
    out = np.asarray(target_rad, dtype=np.float32).copy()
    for p_idx, r_idx in ((4, 5), (10, 11)):
        p = float(np.clip(out[p_idx], pitch_range[0], pitch_range[1]))
        r = float(np.clip(out[r_idx], roll_range[0], roll_range[1]))
        if coupled_envelope:
            # Keep extreme combined pitch+roll away from closed-chain linkage singularity.
            norm = abs(p) / 1.22 + abs(r) / 0.65
            if norm > 1.0:
                p /= norm
                r /= norm
        out[p_idx] = p
        out[r_idx] = r
    return out


class BPlusStepRunner:
    """Stateful 50 Hz ONNX runner for `bplus_step.onnx` (`B_PLUS` router + `SafeFall` Stage II)."""

    def __init__(self, bundle_dir: str | Path):
        self.bundle_dir = Path(bundle_dir)
        self.cfg = yaml.safe_load((self.bundle_dir / "config.yaml").read_text())
        if self.cfg.get("format") != "bplus_sdk_sim_v1" or int(self.cfg.get("policy_hz", 0)) != 50:
            raise ValueError(f"Incompatible B_PLUS bundle config in {self.bundle_dir}")
        default_q = np.asarray(self.cfg["default_q"], dtype=np.float32)
        if default_q.shape != (29,):
            raise ValueError(f"Expected 29D default_q, got {default_q.shape}")
        self.default_q = default_q
        opt = ort.SessionOptions()
        opt.intra_op_num_threads = 1
        opt.inter_op_num_threads = 1
        opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            str(self.bundle_dir / "bplus_step.onnx"), opt, providers=["CPUExecutionProvider"]
        )
        self.reset()

    def reset(self) -> None:
        self.previous = self.default_q[None, :].copy()
        self.history = np.zeros((1, 475), dtype=np.float32)
        self.hidden = np.zeros((1, 1, 128), dtype=np.float32)
        self.memory = np.zeros((1, 4), dtype=np.float32)
        self.alpha = 0.0
        self.destination = 0.0
        self.probability = 0.0
        self.frame = np.zeros((1, 133), dtype=np.float32)
        self.first = True

    def infer(
        self,
        sensors_65: np.ndarray,
        pelvis_acc_3: np.ndarray,
        c_target_29: np.ndarray,
    ) -> np.ndarray:
        s65 = np.asarray(sensors_65, dtype=np.float32).reshape(1, 65)
        acc = np.asarray(pelvis_acc_3, dtype=np.float32).reshape(1, 3)
        ct = np.asarray(c_target_29, dtype=np.float32).reshape(1, 29)
        sensors = np.concatenate((s65, acc), axis=-1)
        if not (np.isfinite(sensors).all() and np.isfinite(ct).all()):
            raise RuntimeError("Nonfinite B_PLUS sensor or c_target input")
        outputs = self.session.run(
            None,
            {
                "sensors": sensors,
                "c_target": ct,
                "previous_target": self.previous,
                "history": self.history,
                "hidden": self.hidden,
                "memory": self.memory,
                "alpha": np.array([[self.alpha]], dtype=np.float32),
                "destination": np.array([[self.destination]], dtype=np.float32),
                "reset": np.array([[self.first]], dtype=bool),
            },
        )
        if not all(np.isfinite(x).all() for x in outputs):
            raise RuntimeError("Nonfinite B_PLUS ONNX output")
        target, next_hist, next_hid, next_mem, next_alpha, next_dest, prob, frame = outputs
        self.previous = target.astype(np.float32, copy=True)
        self.history = next_hist.astype(np.float32, copy=True)
        self.hidden = next_hid.astype(np.float32, copy=True)
        self.memory = next_mem.astype(np.float32, copy=True)
        self.alpha = float(next_alpha[0, 0])
        self.destination = float(next_dest[0, 0])
        self.probability = float(prob[0, 0])
        self.frame = frame.astype(np.float32, copy=True)
        self.first = False
        return self.previous[0].copy()


class AutonomousBridgeClock:
    """Exact NumPy mirror of `VoluntaryOwner` + upstream `B_PLUS` preemption."""

    def __init__(self, *, decision_steps: int = 5, transition_steps: int = 5, dt: float = 0.02):
        if decision_steps < 1 or transition_steps < 1:
            raise ValueError("decision_steps and transition_steps must be >= 1")
        self.decision = int(decision_steps)
        self.transition = int(transition_steps)
        self.dt = float(dt)
        self.reset()

    def reset(self, *, active_immediately: bool = False, previous_raw: np.ndarray | None = None) -> None:
        self.prev_alpha = 0.0
        self.triggers = 0
        self._init_option(released=not active_immediately, previous_raw=previous_raw)

    def _init_option(self, *, released: bool, previous_raw: np.ndarray | None) -> None:
        self.age = 0
        self.released = bool(released)
        self.started = False
        self.fading = False
        self.fade_count = 0
        self.request_time = float("nan")
        self.release_time = float("nan")
        if previous_raw is None:
            self.last_raw = None
            self.anchor = None
        else:
            raw = np.asarray(previous_raw, dtype=np.float32).reshape(1, 29).copy()
            self.last_raw = raw
            self.anchor = raw.copy()

    def on_protection_alpha(self, alpha: float, previous_raw: np.ndarray) -> bool:
        """Update ownership state from `B_PLUS` alpha. Returns True if a NEW bridge option just started."""
        if not math.isfinite(alpha) or alpha < 0.0 or alpha > 1.0:
            raise ValueError(f"Invalid protection authority alpha={alpha}")
        started_new = False
        if alpha > 0.0:
            # Immediate preemption: SafeFall retakes full priority.
            self.released = True
            self.fading = False
        elif self.prev_alpha > 0.0:
            self.triggers += 1
            self._init_option(released=False, previous_raw=previous_raw)
            started_new = True
        self.prev_alpha = float(alpha)
        return started_new

    @property
    def owns_command(self) -> bool:
        return not self.released

    def eligible(self) -> bool:
        return (not self.released) and (not self.fading) and (self.age % self.decision == 0)

    def step(self, request: bool, proposed_raw: np.ndarray, sdk_raw: np.ndarray) -> tuple[np.ndarray, bool]:
        """Advance one 50 Hz option tick; matches `VoluntaryOwner.step` bit-for-bit."""
        prop = np.asarray(proposed_raw, dtype=np.float32).reshape(1, 29)
        sdk = np.asarray(sdk_raw, dtype=np.float32).reshape(1, 29)
        start = self.eligible() and bool(request)
        skip = start and (not self.started)
        if start:
            self.request_time = self.age * self.dt
        if skip:
            self.release_time = self.age * self.dt
            self.released = True
        if self.last_raw is None:
            self.last_raw = sdk.copy()
            self.anchor = sdk.copy()
        if start and (not skip):
            self.anchor = self.last_raw.copy()
            self.fading = True
        active = (not self.released) and (not self.fading)
        if self.fading:
            self.fade_count += 1
        fade_alpha = float(np.clip(1.0 - self.fade_count / float(self.transition), 0.0, 1.0))
        fade = self.anchor * fade_alpha + sdk * (1.0 - fade_alpha)
        command = prop.copy() if active else sdk.copy()
        if self.fading:
            command = fade.astype(np.float32, copy=False)
        finished = self.fading and (self.fade_count >= self.transition)
        if finished:
            self.release_time = (self.age + 1) * self.dt
            self.released = True
            self.fading = False
        self.started = self.started or active
        self.last_raw = command.astype(np.float32, copy=True)
        self.age += 1
        return command.reshape(29), active


class AutonomousGroundBridgeRunner:
    """Stateful 50 Hz ONNX runner for `autonomous_ground_bridge.onnx` + `AutonomousBridgeClock`."""

    def __init__(self, bundle_dir: str | Path, sdk_contract: dict[str, Any]):
        self.bundle_dir = Path(bundle_dir)
        self.cfg = yaml.safe_load((self.bundle_dir / "config.yaml").read_text())
        if self.cfg.get("format") != "sdk_autonomous_ground_bridge_native_v1" or int(self.cfg.get("policy_hz", 0)) != 50:
            raise ValueError(f"Incompatible GroundBridge bundle config in {self.bundle_dir}")
        if int(self.cfg.get("memory_dim", 0)) != 122:
            raise ValueError("Expected 122D GroundBridge causal memory")
        for key in ("offset", "scale", "kp", "kd", "effort"):
            a = np.asarray(self.cfg[key], dtype=np.float32)
            b = np.asarray(sdk_contract[key], dtype=np.float32)
            if a.shape != (29,) or b.shape != (29,) or not np.allclose(a, b, rtol=0.0, atol=1e-6):
                raise ValueError(f"GroundBridge / SDK-A motor contract differs on {key}")
        self.offset = np.asarray(self.cfg["offset"], dtype=np.float32)
        self.scale = np.asarray(self.cfg["scale"], dtype=np.float32)
        self.release_threshold = float(self.cfg["release_threshold"])
        self.clock = AutonomousBridgeClock(
            decision_steps=int(self.cfg["decision_steps"]),
            transition_steps=int(self.cfg["transition_steps"]),
            dt=0.02,
        )
        opt = ort.SessionOptions()
        opt.intra_op_num_threads = 1
        opt.inter_op_num_threads = 1
        opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            str(self.bundle_dir / "autonomous_ground_bridge.onnx"),
            opt,
            providers=["CPUExecutionProvider"],
        )
        self.reset()

    def reset(self) -> None:
        self.memory = np.zeros((1, 122), dtype=np.float32)
        self.last_release_probability = 0.0
        self.clock.reset()

    def reset_option_memory(self) -> None:
        self.memory.fill(0.0)
        self.last_release_probability = 0.0

    def step(self, sdk_history_884: np.ndarray, sdk_raw_29: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool, bool]:
        hist = np.asarray(sdk_history_884, dtype=np.float32).reshape(1, 4, 221)
        sdk_raw = np.asarray(sdk_raw_29, dtype=np.float32).reshape(1, 29)
        if not (np.isfinite(hist).all() and np.isfinite(sdk_raw).all()):
            raise RuntimeError("Nonfinite GroundBridge history or sdk_action input")
        proprio = np.ascontiguousarray(hist[:, :, 128:], dtype=np.float32)
        outputs = self.session.run(
            None,
            {
                "proprio_history": proprio,
                "sdk_action": sdk_raw,
                "memory": self.memory,
            },
        )
        if not all(np.isfinite(x).all() for x in outputs):
            raise RuntimeError("Nonfinite GroundBridge ONNX output")
        _, raw_action, release_prob, next_memory = outputs
        self.memory = next_memory.astype(np.float32, copy=True)
        self.last_release_probability = float(release_prob[0, 0])
        request = self.last_release_probability >= self.release_threshold
        was_fading_before = self.clock.fading
        cmd_raw, active = self.clock.step(request, raw_action[0], sdk_raw[0])
        fading_now = (not active) and (was_fading_before or self.clock.fading)
        target = (self.offset + self.scale * cmd_raw).astype(np.float32)
        return target, cmd_raw.astype(np.float32), active, fading_now


class ProtectionStackRuntime:
    """Unified 4-component coordinator wrapping `B_PLUS` + optional `AutonomousGroundBridge`."""

    def __init__(
        self,
        sdk_contract: dict[str, Any],
        *,
        bplus_bundle: str | Path,
        ground_bridge_bundle: str | Path | None = None,
        clamp_parallel_ankles: bool = False,
    ):
        self.offset = np.asarray(sdk_contract["offset"], dtype=np.float32)
        self.scale = np.asarray(sdk_contract["scale"], dtype=np.float32)
        self.bplus = BPlusStepRunner(bplus_bundle)
        self.bridge = (
            AutonomousGroundBridgeRunner(ground_bridge_bundle, sdk_contract)
            if ground_bridge_bundle is not None
            else None
        )
        self.clamp_parallel_ankles = bool(clamp_parallel_ankles)
        self.reset()

    def reset(self) -> None:
        self.bplus.reset()
        if self.bridge is not None:
            self.bridge.reset()
        self.last_telemetry: dict[str, Any] = {
            "owner": "SDK_A",
            "alpha": 0.0,
            "probability": 0.0,
            "bridge_active": False,
            "bridge_fading": False,
            "bridge_release_prob": 0.0,
            "bridge_triggers": 0,
        }

    def step(
        self,
        *,
        sensors_65: np.ndarray,
        pelvis_acc_3: np.ndarray,
        sdk_target_29: np.ndarray,
        sdk_raw_29: np.ndarray,
        sdk_history_884: np.ndarray,
        previous_executed_raw_29: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        """Run one 50 Hz closed-loop coordination tick.

        Returns `(executed_target_29, executed_raw_29_for_sdk_next_tick, telemetry_dict)`.
        """
        target = np.asarray(sdk_target_29, dtype=np.float32).reshape(29).copy()
        exec_raw = np.asarray(sdk_raw_29, dtype=np.float32).reshape(29).copy()

        bplus_target = self.bplus.infer(sensors_65, pelvis_acc_3, target)
        alpha = self.bplus.alpha
        owner = "SDK_A"
        bridge_active = False
        bridge_fading = False
        bridge_prob = 0.0

        if alpha > 0.0:
            target = bplus_target
            exec_raw = ((target - self.offset) / self.scale).astype(np.float32)
            owner = "B_PLUS_SAFEFALL"

        if self.bridge is not None:
            started_new = self.bridge.clock.on_protection_alpha(alpha, previous_executed_raw_29)
            if started_new:
                self.bridge.reset_option_memory()
            if self.bridge.clock.owns_command:
                # Pass SDK-A's unexecuted proposal `sdk_raw_29` into GroundBridge features,
                # then synchronize BOTH SDK-A's raw action and B_PLUS's previous target!
                gb_target, gb_raw, bridge_active, bridge_fading = self.bridge.step(sdk_history_884, sdk_raw_29)
                bridge_prob = self.bridge.last_release_probability
                if bridge_active or bridge_fading:
                    target = gb_target
                    exec_raw = gb_raw
                    self.bplus.previous[0, :] = target
                    owner = "GROUND_BRIDGE" if bridge_active else "GROUND_BRIDGE_FADE"

        if self.clamp_parallel_ankles and owner != "SDK_A":
            target = clamp_tiangong3_parallel_ankles(target)
            exec_raw = ((target - self.offset) / self.scale).astype(np.float32)
            self.bplus.previous[0, :] = target

        self.last_telemetry = {
            "owner": owner,
            "alpha": float(alpha),
            "probability": float(self.bplus.probability),
            "bridge_active": bool(bridge_active),
            "bridge_fading": bool(bridge_fading),
            "bridge_release_prob": float(bridge_prob),
            "bridge_triggers": int(self.bridge.clock.triggers) if self.bridge is not None else 0,
        }
        return target, exec_raw, dict(self.last_telemetry)
