"""Offline configuration, kp/kd alignment, and torque-limit protection inspection.
Does not create a ROS node or publish commands.
"""
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml

root = Path(__file__).resolve().parents[1]
project_root = root.parent
if str(root) not in sys.path:
    sys.path.insert(0, str(root))
if str(root / "common" / "lib") not in sys.path:
    sys.path.insert(0, str(root / "common" / "lib"))

from common.body_id_map import BodyServoIdMap
from common.torque_limiter import (
    clamp_parallel_ankle_torque,
    clamp_pd_target_by_torque,
    enforce_wire_float32_torque_limit,
    load_sim_effort_limits,
)
import sptlib_python as spt

c = yaml.safe_load((root / "config/sdk_a_hardware.yaml").read_text())
p = root / "policy/sdk_a_to"
m = json.loads((p / "manifest.json").read_text())
d = json.loads((p / "deploy.json").read_text())

assert not c["sim"] and not c["sdk_a_autostart"] and c["dt"] == 0.01
dex_amp_cfg = yaml.safe_load((root / "policy/dex_amp/config/dex_amp.yaml").read_text())
assert dex_amp_cfg["dt"] == 0.01 and dex_amp_cfg["control"]["decimation"] == 2, "DEXAMP must align with Dknt0/xmigcs_dex3 (100Hz loop, decimation=2)"
assert hashlib.sha256((p / "policy.onnx").read_bytes()).hexdigest() == m["onnx_sha256"]
assert hashlib.sha256((p / "deploy.json").read_bytes()).hexdigest() == m["contract_sha256"]
assert all(len(d[k]) == 29 for k in ["joint_names", "kp", "kd", "effort", "offset", "scale"])

# 1. Verify CAN ID ordering & 29-joint mapping
id_map = BodyServoIdMap()
id_map.body_can_id_map_init()
expected_can_ids = [
    51, 52, 53, 54, 55, 56,
    61, 62, 63, 64, 65, 66,
    33, 32, 31,
    11, 12, 13, 14, 15, 16, 17,
    21, 22, 23, 24, 25, 26, 27,
]
actual_can_ids = [id_map.get_id_by_index(i) for i in range(29)]
assert actual_can_ids == expected_can_ids, f"CAN ID mismatch: {actual_can_ids}"

# 2. Verify simulation scene XML ctrlrange & actuatorfrcrange match deploy.json['effort']
scene_xml = project_root / m["scene"]
tree = ET.parse(scene_xml)
actuators = {
    el.attrib["name"]: el.attrib
    for el in tree.getroot().findall(".//actuator/motor")
}
kp = np.asarray(d["kp"], dtype=np.float64)
kd = np.asarray(d["kd"], dtype=np.float64)
effort = np.asarray(d["effort"], dtype=np.float64)
for idx, jname in enumerate(d["joint_names"]):
    assert jname in actuators, f"Missing actuator {jname} in {scene_xml}"
    ctrl_lo, ctrl_hi = map(float, actuators[jname]["ctrlrange"].split())
    j_el = tree.getroot().find(f".//joint[@name='{jname}']")
    assert j_el is not None, f"Missing joint {jname} in {scene_xml}"
    frc_lo, frc_hi = map(float, j_el.attrib["actuatorfrcrange"].split())
    assert np.isclose(ctrl_lo, -effort[idx]) and np.isclose(ctrl_hi, effort[idx]), (
        f"{jname} ctrlrange ({ctrl_lo}, {ctrl_hi}) != ±{effort[idx]}"
    )
    assert np.isclose(frc_lo, -effort[idx]) and np.isclose(frc_hi, effort[idx]), (
        f"{jname} actuatorfrcrange ({frc_lo}, {frc_hi}) != ±{effort[idx]}"
    )

# 3. Verify kp/kd alignment across FIX and ZERO hardware states
fix_cfg = json.loads((root / "policy/fix/config.json").read_text())
assert np.allclose(fix_cfg["kp"], kp) and np.allclose(fix_cfg["kd"], kd), "FIX kp/kd mismatch with deploy.json"
assert c.get("sdk_a_hardware", False) is True, "sdk_a_hardware must be true so ZERO uses deploy.json kp/kd/offset"

# 4. Verify hardware torque-limit configuration is active
ri_cfg = c["robot_interface"]
sdka_cfg = c["sdk_a"]
assert ri_cfg.get("torque_limit_enabled") is True, "robot_interface.torque_limit_enabled must be true"
assert 0.0 < float(ri_cfg.get("torque_limit_scale", 1.0)) <= 1.0
assert "SDK_A" in ri_cfg.get("torque_limit_states", []), "SDK_A must be protected in torque_limit_states"
assert sdka_cfg.get("torque_limit_enabled", True) is True, "sdk_a.torque_limit_enabled must be true"
assert 0.0 < float(sdka_cfg.get("torque_limit_scale", 1.0)) <= 1.0
loaded_effort = load_sim_effort_limits(scale=float(ri_cfg.get("torque_limit_scale", 1.0)))
assert np.allclose(loaded_effort, effort * float(ri_cfg.get("torque_limit_scale", 1.0)))

# 5. Verify torque limiter math (unsaturated identity + extreme saturation + parallel ankle sptlib + float32 wire)
rng = np.random.default_rng(20261111)
q_a = np.asarray(d["offset"], dtype=np.float64)
dq_a_zero = np.zeros(29, dtype=np.float64)
small_q_d = q_a + 0.05 * (effort / kp)
q_d_unchanged, _, _, _, mask_small = clamp_pd_target_by_torque(small_q_d, q_a, dq_a_zero, kp, kd, loaded_effort)
assert not np.any(mask_small) and np.array_equal(q_d_unchanged, small_q_d), "Unsaturated target was perturbed!"

for _ in range(32):
    dq_a = rng.uniform(-8.0, 8.0, size=29)
    extreme_q_d = q_a + rng.uniform(-5.0, 5.0, size=29)
    q_d_safe, tau_d_safe, tau_raw, tau_clip, _ = clamp_pd_target_by_torque(
        extreme_q_d, q_a, dq_a, kp, kd, loaded_effort
    )
    tau_check = kp * (q_d_safe - q_a) - kd * dq_a + tau_d_safe
    assert np.all(np.abs(tau_check) <= loaded_effort + 1e-9), "Serial PD torque exceeded effort limit!"

    # Test parallel ankle s2p + wire float32 quantization
    s2p = spt.funcSPTrans()
    p_est = rng.uniform(-0.2, 0.2, size=4)
    dp_est = rng.uniform(-2.0, 2.0, size=4)
    s2p.set_p_est(p_est, dp_est, np.zeros(4))
    s2p.calcFK()
    s2p.calcIK()
    ok_s, q_as4, dq_as4, _ = s2p.get_s_state()
    assert ok_s
    ankle_idx = np.array([4, 5, 10, 11])
    ankle_lim = loaded_effort[ankle_idx]
    tor_s4 = np.clip(
        kp[ankle_idx] * (extreme_q_d[ankle_idx] - q_as4) - kd[ankle_idx] * dq_as4,
        -ankle_lim * (1.0 - 1e-6),
        ankle_lim * (1.0 - 1e-6),
    )
    q_d_s_ik = q_d_safe[ankle_idx].copy()
    for p_i, r_i in ((0, 1), (2, 3)):
        p_val = float(np.clip(q_d_s_ik[p_i], -1.10, 0.48))
        r_val = float(np.clip(q_d_s_ik[r_i], -0.45, 0.45))
        env_norm = abs(p_val) / 1.10 + abs(r_val) / 0.52
        if env_norm > 1.0:
            p_val /= env_norm
            r_val /= env_norm
        q_d_s_ik[p_i] = p_val
        q_d_s_ik[r_i] = r_val
    s2p.set_s_des(q_d_s_ik, np.zeros(4), tor_s4)
    s2p.calc_joint_pos_ref()
    s2p.calc_joint_tor_des()
    ok_p, _, qdot_dp4, tor_dp4 = s2p.get_p_des()
    assert ok_p
    tor_dp4, _ = clamp_parallel_ankle_torque(tor_dp4, ankle_lim)
    assert np.all(np.abs(tor_dp4) <= ankle_lim), "Parallel ankle torque exceeded 55 Nm limit!"

    # Full 29-motor wire float32 check
    ankle_kp_p = np.asarray(ri_cfg["ankle_kp_p"], dtype=np.float64)
    ankle_kd_p = np.asarray(ri_cfg["ankle_kd_p"], dtype=np.float64)
    q_a_p = q_a.copy()
    dq_a_p = dq_a.copy()
    q_a_p[ankle_idx] = p_est
    dq_a_p[ankle_idx] = dp_est
    q_d_p = q_d_safe.copy()
    dq_d_p = np.zeros(29, dtype=np.float64)
    tau_d_p = tau_d_safe.copy()
    kp_p = kp.copy()
    kd_p = kd.copy()
    q_d_p[ankle_idx] = (tor_dp4 - ankle_kd_p * (qdot_dp4 - dp_est)) / ankle_kp_p + p_est
    dq_d_p[ankle_idx] = qdot_dp4
    tau_d_p[ankle_idx] = 0.0
    kp_p[ankle_idx] = ankle_kp_p
    kd_p[ankle_idx] = ankle_kd_p

    motor_dir = rng.choice([-1.0, 1.0], size=29)
    zero_pos = np.asarray(ri_cfg["zero_pos"], dtype=np.float64)
    zero_offset = np.zeros(29, dtype=np.float64)
    zero_cnt = np.zeros(29, dtype=np.float64)
    q_d_motor = (q_d_p - zero_offset) * motor_dir + zero_pos
    dq_d_motor = dq_d_p * motor_dir
    tau_d_motor = tau_d_p * motor_dir
    enforce_wire_float32_torque_limit(
        q_d_motor, dq_d_motor, tau_d_motor,
        q_a_p, dq_a_p, kp_p, kd_p,
        zero_offset, zero_cnt, motor_dir, zero_pos, loaded_effort,
    )
    pos_a_f32 = ((q_a_p - zero_offset) * motor_dir + zero_pos).astype(np.float32)
    spd_a_f32 = (dq_a_p * motor_dir).astype(np.float32)
    tau_wire_f32 = (
        kp_p.astype(np.float32).astype(np.float64) * (q_d_motor.astype(np.float32).astype(np.float64) - pos_a_f32.astype(np.float64))
        + kd_p.astype(np.float32).astype(np.float64) * (dq_d_motor.astype(np.float32).astype(np.float64) - spd_a_f32.astype(np.float64))
        + tau_d_motor.astype(np.float32).astype(np.float64)
    )
    assert np.all(np.abs(tau_wire_f32) <= loaded_effort), (
        f"Wire float32 torque exceeded effort limit: max ratio={np.max(np.abs(tau_wire_f32) / loaded_effort)}"
    )

# 6. Verify FSMStateSDKA 100Hz outer loop + 50Hz TickDecimator(2) alignment
from common.robot_data import RobotData
from common.joystick import ControlFlag
from policy.sdk_a_to.fsm_sdk_a import FSMStateSDKA

rd_test = RobotData(29, 35)
rd_test.config_file_ = str(root / "config/sdk_a_hardware.yaml")
rd_test.control_dt_ = float(c["dt"])
rd_test.q_a_s_[6:] = q_a
fsm_sdka = FSMStateSDKA(rd_test)
assert fsm_sdka.decimation == 2 and np.isclose(fsm_sdka.decimation * fsm_sdka.control_dt, 0.02), (
    f"Expected 100Hz control loop + decimation=2 (50Hz policy), got control_dt={fsm_sdka.control_dt}, decimation={fsm_sdka.decimation}"
)
flag_test = ControlFlag()
for tick_idx in range(6):
    # Perturb joint velocity on hold ticks to confirm 100Hz torque-limit recomputation
    rd_test.q_dot_a_s_[6:] = 0.0 if (tick_idx % 2 == 0) else 0.5
    fsm_sdka.run(flag_test)
assert fsm_sdka.counter == 3, f"Expected 3 policy inferences across 6 ticks (50Hz in 100Hz loop), got {fsm_sdka.counter}"

s = ort.InferenceSession(str(p / "policy.onnx"), providers=["CPUExecutionProvider"])
print("PASS: ONNX/contract hashes; 29 joints CAN/XML kp/kd/effort aligned; 100Hz loop + 50Hz policy (decimation=2); hardware serial+parallel-ankle torque limiter verified; STOP startup.")
print("Inputs:", [(x.name, x.shape) for x in s.get_inputs()])
print("Torque limits (Nm):", [float(x) for x in loaded_effort])
print("IMU hardware mounting verified:", c["sdk_a"]["imu_mount_verified"])
print("IMU input contract:", c["sdk_a"].get("imu_input_frame"))
if "--require-calibration" in sys.argv and c["sdk_a"].get("imu_input_frame") != "sdk_body_euler" and not c["sdk_a"]["imu_mount_verified"]:
    raise SystemExit("Hardware mounting not yet verified: edit config/sdk_a_hardware.yaml after measuring the IMU frame.")
