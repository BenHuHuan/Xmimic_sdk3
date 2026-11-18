"""RaiSim physics + Rayrai 3D renderer worker process.

Runs in the RaiSim Python environment (e.g. raisim2Lib/.venv) and exchanges
motor targets, robot state, suspension controls, and HUD telemetry with
RaisimBridge over shared mmap + UNIX socket.
"""
from pathlib import Path
import argparse
import ctypes
import ctypes.util
import json
import os
import socket
import struct
import sys
import time
import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from sim_suspension import SimSuspension

# Shared float64 layout offsets (must match raisim_bridge.py)
SH_ALIVE = 0
SH_SEQ = 1
SH_TIME = 2
SH_FPS = 3
SH_PAUSED = 4
SH_SUSP_EN = 5
SH_SUSP_Z = 6
SH_SUSP_K = 7
SH_NONFOOT = 8
SH_PENETRATION = 9
SH_RESET_REQ = 10
SH_FSM_STATE = 11
SH_CTRL_HZ = 12
SH_FLT_ALPHA = 13
SH_JITTER_PCT = 14
SH_TWITCH_PCT = 15
SH_PUSH_VEC = 16  # 3 floats [fx, fy, fz]
SH_PUSH_ACTIVE = 19
SH_ACC = 20       # 3 floats [ax, ay, az]
SH_QPOS = 32      # 36 floats
SH_QVEL = 68      # 35 floats
SH_TAU = 103      # 29 floats
SH_QDES = 132     # 29 floats
SH_DQDES = 161    # 29 floats
SH_KP = 190       # 29 floats
SH_KD = 219       # 29 floats
SH_TAUFF = 248    # 29 floats
SHARED_LEN = 277

# Binary control ops: b'S' (step), b'R' (reset), b'H' (hang), b'P' (pause toggle), b'Q' (quit)
_STATE_NAMES = {0: 'STOP', 1: 'ZERO', 2: 'SDK-A', 3: 'WALK', 4: 'FIX'}


def _quat_wxyz_to_mat(q):
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def _quat_from_z_to_vec(vec):
    norm = float(np.linalg.norm(vec))
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    v = vec / norm
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    dot = float(np.dot(z, v))
    if dot > 1.0 - 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if dot < -1.0 + 1e-8:
        return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    axis = np.cross(z, v)
    w = 1.0 + dot
    q = np.array([w, axis[0], axis[1], axis[2]], dtype=np.float64)
    return q / np.linalg.norm(q)


def find_activation_key(explicit_key=None, raisim_root=None):
    if explicit_key:
        p = Path(explicit_key).expanduser()
        return str(p.resolve()) if p.is_file() else None
    env_key = os.environ.get('RAISIM_ACTIVATION_KEY')
    if env_key is not None:
        if not env_key.strip():
            return None
        p = Path(env_key).expanduser()
        return str(p.resolve()) if p.is_file() else None
    candidates = [
        Path.home() / '.raisim' / 'activation.raisim',
        Path.home() / 'raisim' / 'activation.raisim',
        Path.home() / 'Downloads' / 'activation.raisim',
    ]
    if raisim_root:
        candidates.append(Path(raisim_root) / 'rsc' / 'activation.raisim')
    for path in candidates:
        if path.is_file():
            return str(path.resolve())
    return None


class _SDLKeyPoller:
    """Edge-triggered SDL2 keyboard poller active only when RayraiWindow has focus."""

    def __init__(self):
        self._sdl = None
        self._prev = {}
        self._bindings = {}
        lib = ctypes.util.find_library('SDL2')
        if not lib:
            return
        try:
            sdl = ctypes.CDLL(lib)
            sdl.SDL_GetKeyboardFocus.restype = ctypes.c_void_p
            sdl.SDL_GetKeyboardState.argtypes = [ctypes.POINTER(ctypes.c_int)]
            sdl.SDL_GetKeyboardState.restype = ctypes.POINTER(ctypes.c_uint8)
            sdl.SDL_GetScancodeFromName.argtypes = [ctypes.c_char_p]
            sdl.SDL_GetScancodeFromName.restype = ctypes.c_int
            self._sdl = sdl
            key_actions = {
                'F9': 'ON', 'Insert': 'ON', '9': 'ON',
                'F10': 'OFF', 'Delete': 'OFF', '0': 'OFF',
                'Keypad 0': 'TOGGLE', 'Keypad 5': 'TOGGLE', 'H': 'TOGGLE',
                'Up': 'UP', '7': 'UP', 'Keypad 7': 'UP', 'Keypad 9': 'UP', 'PageUp': 'UP',
                'Down': 'DOWN', '8': 'DOWN', 'Keypad 1': 'DOWN', 'Keypad 3': 'DOWN', 'PageDown': 'DOWN',
                '.': 'STRONGER', 'Keypad +': 'STRONGER', 'Keypad 8': 'STRONGER', 'Keypad 6': 'STRONGER',
                ',': 'WEAKER', 'Keypad -': 'WEAKER', 'Keypad 2': 'WEAKER', 'Keypad 4': 'WEAKER',
                'Space': 'PAUSE_TOGGLE',
                'R': 'RESET',
            }
            for name, action in key_actions.items():
                sc = int(sdl.SDL_GetScancodeFromName(name.encode('ascii')))
                if sc > 0:
                    self._bindings[sc] = action
                    self._prev[sc] = False
        except Exception:
            self._sdl = None

    def poll(self):
        if self._sdl is None:
            return []
        if not self._sdl.SDL_GetKeyboardFocus():
            for sc in self._prev:
                self._prev[sc] = False
            return []
        num = ctypes.c_int(0)
        state = self._sdl.SDL_GetKeyboardState(ctypes.byref(num))
        if not state:
            return []
        triggered = []
        nkeys = int(num.value)
        for sc, action in self._bindings.items():
            pressed = bool(sc < nkeys and state[sc])
            if pressed and not self._prev[sc]:
                triggered.append(action)
            self._prev[sc] = pressed
        return triggered


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shared', type=Path, required=True)
    p.add_argument('--sock', required=True)
    p.add_argument('--urdf', required=True)
    p.add_argument('--meta', type=Path, required=True)
    p.add_argument('--dt', type=float, default=0.001)
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--renderer', choices=('rayrai', 'tcp'), default='rayrai')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--raisim-root', default='')
    p.add_argument('--activation-key', default='')
    args = p.parse_args()

    try:
        import raisimpy as raisim
    except ImportError as exc:
        raise RuntimeError(
            f'Failed to import raisimpy ({exc}). Install/build raisim2Lib with `-DRAISIM_PY=ON` '
            'and set `RAISIM2LIB_ROOT`, or run with `--physics mujoco`.'
        ) from exc

    key_path = find_activation_key(args.activation_key or None, args.raisim_root or None)
    if key_path is None:
        raise RuntimeError(
            'RaiSim activation key not found (~/.raisim/activation.raisim or RAISIM_ACTIVATION_KEY). '
            'Install a valid activation.raisim or run with `--physics mujoco`.'
        )
    raisim.World.setActivationKey(key_path)

    meta = json.loads(args.meta.read_text())
    joint_names = meta['joint_names']
    effort_limits = np.asarray(meta['effort'], dtype=np.float64)
    joint_info = meta['joint_info']
    excluded_pairs = meta['excluded_pairs']
    imu_site_pos_b = np.array([-0.081641, -0.00010952, -0.063057], dtype=np.float64)

    world = raisim.World()
    world.setTimeStep(float(args.dt))
    ground = world.addGround()

    options = raisim.ArticulatedSystemOption()
    options.convexify_collision_meshes = False
    options.do_not_collide_with_parent = True
    robot = world.addArticulatedSystem(args.urdf, meta.get('mesh_dir', ''), joint_names, options=options)
    robot.setName('tiangong3')

    actuated_names = [n for n in robot.getMovableJointNames() if n != 'ROOT']
    if actuated_names != list(joint_names):
        raise RuntimeError(
            f'RaiSim joint order mismatch: {actuated_names} vs {joint_names}'
        )

    movable_bodies = set(robot.getBodyNames())
    for b1, b2 in excluded_pairs:
        if b1 in movable_bodies and b2 in movable_bodies and b1 != b2:
            idx1 = robot.getBodyIdx(b1)
            idx2 = robot.getBodyIdx(b2)
            if idx1 >= 0 and idx2 >= 0 and idx1 != idx2:
                robot.ignoreCollisionBetween(idx1, idx2)

    dof = robot.getDOF()  # 6 + 29 = 35
    armatures = np.zeros(dof, dtype=np.float64)
    dampings = np.zeros(dof, dtype=np.float64)
    act_upper = np.full(dof, 1e6, dtype=np.float64)
    act_lower = np.full(dof, -1e6, dtype=np.float64)
    for i, jn in enumerate(joint_names):
        ji = joint_info[jn]
        armatures[6 + i] = float(ji.get('armature', 0.0))
        dampings[6 + i] = float(ji.get('damping', 0.0))
        eff = float(effort_limits[i])
        act_upper[6 + i] = eff
        act_lower[6 + i] = -eff
    robot.setRotorInertia(armatures)
    robot.setJointDamping(dampings)
    robot.setActuationLimits(act_upper, act_lower)
    robot.setControlMode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)

    waist_body_idx = robot.getBodyIdx('waist_pitch_link')
    waist_frame_idx = robot.getFrameIdxByLinkName('waist_pitch_link')
    pelvis_frame_idx = robot.getFrameIdxByLinkName('pelvis')
    foot_body_indices = {robot.getBodyIdx('ankle_roll_l_link'), robot.getBodyIdx('ankle_roll_r_link')}
    ground_world_idx = ground.getIndexInWorld()
    robot_world_idx = robot.getIndexInWorld()
    com_b_list = robot.getBodyCOM_B()
    waist_com_b = np.asarray(com_b_list[waist_body_idx], dtype=np.float64) if waist_body_idx >= 0 else np.zeros(3)

    suspension = SimSuspension(float(robot.getTotalMass()))

    a = np.load(args.shared, mmap_mode='r+')
    a[SH_ALIVE] = 1.0
    a[SH_SUSP_K] = float(suspension.stiffness)

    viewer = None
    server = None
    key_poller = None
    sling_anchor_vis = None
    sling_line = None
    sliders = {}

    if args.viewer:
        if args.renderer == 'rayrai' and raisim.hasRayrai:
            rr = raisim.rayrai
            viewer = rr.RayraiWindow(world, 1280, 720, visible=True, title='Xrobot3 · RaiSim Rayrai')
            viewer.setRenderQualityPreset(rr.RenderQualityPreset.Balanced)
            viewer.focusOn(robot)
            sliders['pause'] = viewer.addSlider('Pause (0=Run, 1=Pause)', 0.0, 1.0, 0.0)
            sliders['reset'] = viewer.addSlider('Reset Pose (slide >0.5)', 0.0, 1.0, 0.0)
            sliders['susp_en'] = viewer.addSlider('Suspension (0=OFF, 1=ON)', 0.0, 1.0, 0.0)
            sliders['susp_z'] = viewer.addSlider('Sling Height (m)', 0.3, 2.5, 1.15)
            sliders['susp_k'] = viewer.addSlider('Sling Stiffness (N/m)', 50.0, 5000.0, 1000.0)
            sling_anchor_vis = viewer.addVisualSphere('sling_anchor', 0.04, [0.15, 0.90, 0.35, 0.95])
            sling_anchor_vis.setCastsShadows(False)
            sling_anchor_vis.setColorPassVisible(False)
            sling_line = viewer.addVisualCylinder('sling_tether', 0.008, 1.0, [0.15, 0.90, 0.35, 0.95])
            sling_line.setCastsShadows(False)
            sling_line.setColorPassVisible(False)
            key_poller = _SDLKeyPoller()
        else:
            server = raisim.RaisimServer(world)
            server.launchServer(args.port)

    srv_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv_sock.connect(args.sock)
    srv_sock.setblocking(True)

    p_gains = np.zeros(dof, dtype=np.float64)
    d_gains = np.zeros(dof, dtype=np.float64)
    p_target = np.zeros(robot.getGeneralizedCoordinateDim(), dtype=np.float64)
    v_target = np.zeros(dof, dtype=np.float64)
    ff_force = np.zeros(dof, dtype=np.float64)
    prev_v_world = np.zeros(3, dtype=np.float64)
    prev_omega_world = np.zeros(3, dtype=np.float64)

    def sync_suspension_to_shared():
        a[SH_SUSP_EN] = 1.0 if suspension.enabled else 0.0
        a[SH_SUSP_Z] = float(suspension.target[2])
        a[SH_SUSP_K] = float(suspension.stiffness)
        if viewer is not None and sliders:
            viewer.setSliderValue('Suspension (0=OFF, 1=ON)', 1.0 if suspension.enabled else 0.0)
            viewer.setSliderValue('Sling Height (m)', float(suspension.target[2]) if suspension.enabled else 1.15)
            viewer.setSliderValue('Sling Stiffness (N/m)', float(suspension.stiffness))

    def sync_state_to_shared(dt_step, first=False):
        gc = np.asarray(robot.getGeneralizedCoordinate(), dtype=np.float64)
        gv = np.asarray(robot.getGeneralizedVelocity(), dtype=np.float64)
        R_pelvis = _quat_wxyz_to_mat(gc[3:7])
        qvel_body = gv.copy()
        qvel_body[3:6] = R_pelvis.T @ gv[3:6]

        v_world = gv[0:3]
        omega_world = gv[3:6]
        if first or dt_step <= 0.0:
            a_pelvis_w = np.zeros(3, dtype=np.float64)
            alpha_w = np.zeros(3, dtype=np.float64)
        else:
            a_pelvis_w = (v_world - prev_v_world) / dt_step
            alpha_w = (omega_world - prev_omega_world) / dt_step
        prev_v_world[:] = v_world
        prev_omega_world[:] = omega_world

        r_w = R_pelvis @ imu_site_pos_b
        a_site_w = a_pelvis_w + np.cross(alpha_w, r_w) + np.cross(omega_world, np.cross(omega_world, r_w))
        acc_local = R_pelvis.T @ (a_site_w - np.array([0.0, 0.0, -9.81], dtype=np.float64))

        q_err = a[SH_QDES:SH_QDES + 29] - gc[7:]
        dq_err = a[SH_DQDES:SH_DQDES + 29] - gv[6:]
        tau_meas = np.clip(
            a[SH_TAUFF:SH_TAUFF + 29] + a[SH_KP:SH_KP + 29] * q_err + a[SH_KD:SH_KD + 29] * dq_err,
            -effort_limits,
            effort_limits,
        )

        nonfoot = False
        max_pen = 0.0
        for ct in robot.getContacts():
            if ct.isSelfCollision() or ct.getPairObjectIndex() == robot_world_idx:
                continue
            max_pen = max(max_pen, max(0.0, -float(ct.getDepth())))
            if ct.getPairObjectIndex() == ground_world_idx and ct.getlocalBodyIndex() not in foot_body_indices:
                nonfoot = True

        a[SH_SEQ] += 1.0
        a[SH_TIME] = float(world.getWorldTime())
        a[SH_ACC:SH_ACC + 3] = acc_local
        a[SH_QPOS:SH_QPOS + 36] = gc
        a[SH_QVEL:SH_QVEL + 35] = qvel_body
        a[SH_TAU:SH_TAU + 29] = tau_meas
        a[SH_NONFOOT] = 1.0 if nonfoot else 0.0
        a[SH_PENETRATION] = max_pen
        sync_suspension_to_shared()
        a[SH_SEQ] += 1.0

    def handle_hang_command(cmd):
        pos = np.asarray(robot.getBodyPosition(waist_body_idx), dtype=np.float64)
        suspension.command(cmd, pos)
        sync_suspension_to_shared()

    def apply_forces_and_step():
        p_gains[6:] = a[SH_KP:SH_KP + 29]
        d_gains[6:] = a[SH_KD:SH_KD + 29]
        p_target[7:] = a[SH_QDES:SH_QDES + 29]
        v_target[6:] = a[SH_DQDES:SH_DQDES + 29]
        ff_force[6:] = a[SH_TAUFF:SH_TAUFF + 29]
        robot.setPdGains(p_gains, d_gains)
        robot.setPdTarget(p_target, v_target)
        robot.setGeneralizedForce(ff_force)

        robot.clearExternalForcesAndTorques()
        ext_f = np.zeros(3, dtype=np.float64)
        if a[SH_PUSH_ACTIVE] > 0.5:
            ext_f += a[SH_PUSH_VEC:SH_PUSH_VEC + 3]
        if suspension.enabled:
            waist_pos = np.asarray(robot.getBodyPosition(waist_body_idx), dtype=np.float64)
            waist_vel = np.asarray(robot.getFrameVelocity(waist_frame_idx), dtype=np.float64)
            ext_f += suspension.force(waist_pos, waist_vel)
        if np.any(ext_f):
            robot.setExternalForce(
                waist_body_idx,
                raisim.ArticulatedSystem.Frame.WORLD_FRAME,
                ext_f,
                raisim.ArticulatedSystem.Frame.BODY_FRAME,
                waist_com_b,
            )

        if viewer is not None:
            viewer.integrateWorldThreadSafe()
        elif server is not None:
            server.integrateWorldThreadSafe()
        else:
            world.integrate()

    def poll_viewer_ui():
        if viewer is None:
            return True
        if not viewer.render():
            return False
        # Poll ImGui sliders
        slider_pause = viewer.getSlider(sliders['pause']) >= 0.5
        if slider_pause != (a[SH_PAUSED] > 0.5):
            a[SH_PAUSED] = 1.0 if slider_pause else 0.0
        if viewer.getSlider(sliders['reset']) >= 0.5:
            viewer.setSliderValue('Reset Pose (slide >0.5)', 0.0)
            a[SH_RESET_REQ] += 1.0
        slider_en = viewer.getSlider(sliders['susp_en']) >= 0.5
        if slider_en != suspension.enabled:
            handle_hang_command('ON' if slider_en else 'OFF')
        if suspension.enabled:
            sz = float(viewer.getSlider(sliders['susp_z']))
            if abs(sz - suspension.target[2]) > 0.01:
                suspension.target[2] = float(np.clip(sz, 0.3, 2.5))
                a[SH_SUSP_Z] = suspension.target[2]
            sk = float(viewer.getSlider(sliders['susp_k']))
            if abs(sk - suspension.stiffness) > 5.0:
                suspension.stiffness = float(np.clip(sk, 50.0, 5000.0))
                suspension.damping = 1.4 * np.sqrt(suspension.mass * suspension.stiffness)
                a[SH_SUSP_K] = suspension.stiffness

        # Poll focused SDL keyboard hotkeys
        if key_poller is not None:
            for action in key_poller.poll():
                if action == 'PAUSE_TOGGLE':
                    paused = not (a[SH_PAUSED] > 0.5)
                    a[SH_PAUSED] = 1.0 if paused else 0.0
                    viewer.setSliderValue('Pause (0=Run, 1=Pause)', 1.0 if paused else 0.0)
                elif action == 'RESET':
                    a[SH_RESET_REQ] += 1.0
                else:
                    handle_hang_command(action)

        # Update sling visual tether
        if sling_anchor_vis is not None and sling_line is not None:
            if suspension.enabled:
                waist_pos = np.asarray(robot.getBodyPosition(waist_body_idx), dtype=np.float64)
                sling_anchor_vis.setPosition(suspension.target)
                sling_anchor_vis.setColorPassVisible(True)
                delta = suspension.target - waist_pos
                length = float(np.linalg.norm(delta))
                if length > 1e-4:
                    mid = 0.5 * (suspension.target + waist_pos)
                    q_line = _quat_from_z_to_vec(delta)
                    sling_line.setCylinderSize(0.008, length)
                    sling_line.setPosition(mid)
                    sling_line.setOrientation(float(q_line[0]), float(q_line[1]), float(q_line[2]), float(q_line[3]))
                    sling_line.setColorPassVisible(True)
                else:
                    sling_line.setColorPassVisible(False)
            else:
                sling_anchor_vis.setColorPassVisible(False)
                sling_line.setColorPassVisible(False)
        return True

    next_render = time.monotonic()
    last_title = 0.0
    frames = 0
    fps_began = time.monotonic()

    try:
        while True:
            hdr = srv_sock.recv(1)
            if not hdr:
                break
            op = hdr
            if op == b'Q':
                srv_sock.sendall(b'K')
                break
            elif op == b'R':
                gc_init = a[SH_QPOS:SH_QPOS + 36].copy()
                gv_init = a[SH_QVEL:SH_QVEL + 35].copy()
                R_init = _quat_wxyz_to_mat(gc_init[3:7])
                gv_init[3:6] = R_init @ gv_init[3:6]
                robot.setState(gc_init, gv_init)
                robot.clearExternalForcesAndTorques()
                world.setWorldTime(0.0)
                prev_v_world[:] = 0.0
                prev_omega_world[:] = 0.0
                sync_state_to_shared(args.dt, first=True)
                if viewer is not None:
                    if not poll_viewer_ui():
                        a[SH_ALIVE] = 0.0
                srv_sock.sendall(b'K')
            elif op == b'H':
                nlen = struct.unpack('<H', srv_sock.recv(2))[0]
                cmd = srv_sock.recv(nlen).decode('ascii')
                handle_hang_command(cmd)
                srv_sock.sendall(b'K')
            elif op == b'S':
                n_steps = struct.unpack('<I', srv_sock.recv(4))[0]
                if not (a[SH_PAUSED] > 0.5):
                    for _ in range(n_steps):
                        apply_forces_and_step()
                    sync_state_to_shared(args.dt * max(1, n_steps))
                else:
                    sync_suspension_to_shared()
                if viewer is not None:
                    now = time.monotonic()
                    if now >= next_render:
                        if not poll_viewer_ui():
                            a[SH_ALIVE] = 0.0
                            srv_sock.sendall(b'X')
                            break
                        frames += 1
                        next_render = now + 1.0 / 60.0
                        if now - fps_began >= 0.5:
                            a[SH_FPS] = frames / (now - fps_began)
                            frames = 0
                            fps_began = now
                        if now - last_title >= 0.2:
                            st = _STATE_NAMES.get(int(a[SH_FSM_STATE]), 'WAIT')
                            sling = (
                                f'Sling: ENABLED z={a[SH_SUSP_Z]:.2f}m k={a[SH_SUSP_K]:.0f}N/m'
                                if a[SH_SUSP_EN] > 0.5
                                else 'Sling: RELEASED (F9/9/Up=hang, Up/Down=lift/lower, F10/0/Del=release)'
                            )
                            viewer.setTitle(
                                f'Xrobot3 RaiSim | {st} | t={a[SH_TIME]:.1f}s | {a[SH_FPS]:.1f} FPS | {sling}'
                            )
                            last_title = now
                srv_sock.sendall(b'K')
    finally:
        a[SH_ALIVE] = 0.0
        try:
            srv_sock.close()
        except OSError:
            pass
        if viewer is not None:
            viewer.close()
        if server is not None:
            server.killServer()


if __name__ == '__main__':
    main()
