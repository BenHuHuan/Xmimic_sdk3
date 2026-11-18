"""Bridge between ROS2/FastDDS simulator loop and the RaiSim + Rayrai worker."""
from pathlib import Path
import json
import os
import socket
import struct
import subprocess
import sys
import time
import numpy as np

from raisim_urdf import mjcf_to_raisim_urdf
from raisim_worker import (
    SHARED_LEN,
    SH_ACC,
    SH_ALIVE,
    SH_CTRL_HZ,
    SH_DQDES,
    SH_FLT_ALPHA,
    SH_FPS,
    SH_FSM_STATE,
    SH_JITTER_PCT,
    SH_KD,
    SH_KP,
    SH_NONFOOT,
    SH_PAUSED,
    SH_PENETRATION,
    SH_PUSH_ACTIVE,
    SH_PUSH_VEC,
    SH_QDES,
    SH_QPOS,
    SH_QVEL,
    SH_RESET_REQ,
    SH_SUSP_EN,
    SH_SUSP_K,
    SH_SUSP_Z,
    SH_TAU,
    SH_TAUFF,
    SH_TIME,
    SH_TWITCH_PCT,
)


def find_raisim_runtime(project_root):
    """Locate (raisim_root, python_executable, env) for running raisimpy + rayrai."""
    candidates = []
    for key in ('RAISIM2LIB_ROOT', 'RAISIM_LOCAL_INSTALL_ROOT'):
        val = os.environ.get(key)
        if val:
            candidates.append(Path(val).expanduser())
    project_root = Path(project_root).resolve()
    candidates.extend([
        project_root.parent / 'raisim2Lib',
        Path.home() / 'raisim2Lib',
        Path('/media/huan/本地磁盘/raisim2Lib'),
    ])
    raisim_root = None
    for cand in candidates:
        if cand.is_dir() and ((cand / 'raisim').is_dir() or (cand / 'build-py' / 'raisimPy').is_dir()):
            raisim_root = cand.resolve()
            break

    py_candidates = []
    if os.environ.get('RAISIM_PYTHON'):
        py_candidates.append(Path(os.environ['RAISIM_PYTHON']).expanduser())
    if raisim_root is not None:
        py_candidates.append(raisim_root / '.venv' / 'bin' / 'python')
    py_candidates.append(Path(sys.executable))

    python_exe = None
    for py in py_candidates:
        if os.access(py, os.X_OK):
            python_exe = str(py)
            break
    if python_exe is None:
        raise RuntimeError('No executable Python found for RaiSim worker (set RAISIM_PYTHON or RAISIM2LIB_ROOT).')

    env = os.environ.copy()
    # Strip ROS Python 3.10 paths when launching a separate RaiSim venv (e.g. Python 3.12).
    if Path(python_exe).resolve() != Path(sys.executable).resolve():
        env.pop('VIRTUAL_ENV', None)
        py_path = [p for p in env.get('PYTHONPATH', '').split(os.pathsep) if p and '/python3.10/' not in p]
        env['PYTHONPATH'] = os.pathsep.join(py_path)
    if raisim_root is not None:
        ld_parts = [str(raisim_root / 'raisim' / 'lib'), str(raisim_root / 'rayrai' / 'lib')]
        if env.get('LD_LIBRARY_PATH'):
            ld_parts.append(env['LD_LIBRARY_PATH'])
        env['LD_LIBRARY_PATH'] = os.pathsep.join(ld_parts)
        build_py = raisim_root / 'build-py' / 'raisimPy'
        if build_py.is_dir():
            pp = [str(build_py)]
            if env.get('PYTHONPATH'):
                pp.append(env['PYTHONPATH'])
            env['PYTHONPATH'] = os.pathsep.join(pp)
        env['RAISIM_LOCAL_INSTALL_ROOT'] = str(raisim_root)
    return raisim_root, python_exe, env


class RaisimBridge:
    """Synchronous/interactive bridge to a RaiSim + Rayrai worker process."""

    def __init__(
        self,
        scene_xml,
        contract,
        output_dir,
        dt=0.001,
        viewer=False,
        renderer='rayrai',
        port=8080,
        project_root=None,
    ):
        self.output_dir = Path(output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.dt = float(dt)
        self.joint_names = list(contract['joint_names'])
        self.effort_limits = [float(x) for x in contract['effort']]
        self._last_reset_req = 0

        urdf_path = self.output_dir / 'raisim_tiangong3.urdf'
        urdf_meta = mjcf_to_raisim_urdf(scene_xml, urdf_path)
        if urdf_meta['joint_names'] != self.joint_names:
            raise ValueError(
                f"Scene joints {urdf_meta['joint_names']} do not match contract {self.joint_names}"
            )
        meta_path = self.output_dir / 'raisim_meta.json'
        meta_path.write_text(
            json.dumps(
                {
                    'joint_names': self.joint_names,
                    'effort': self.effort_limits,
                    'joint_info': urdf_meta['joint_info'],
                    'excluded_pairs': urdf_meta['excluded_pairs'],
                    'mesh_dir': urdf_meta['mesh_dir'],
                    'total_mass': urdf_meta['total_mass'],
                },
                indent=2,
            )
        )
        self.total_mass = float(urdf_meta['total_mass'])

        self.shared_path = self.output_dir / 'raisim_shared.npy'
        self.shared = np.lib.format.open_memmap(
            self.shared_path, mode='w+', dtype=np.float64, shape=(SHARED_LEN,)
        )
        self.shared[:] = 0.0

        self.sock_path = str(self.output_dir / 'raisim_ctrl.sock')
        Path(self.sock_path).unlink(missing_ok=True)
        self._listen_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._listen_sock.bind(self.sock_path)
        self._listen_sock.listen(1)
        self._listen_sock.settimeout(15.0)

        if project_root is None:
            project_root = Path(__file__).resolve().parents[3]
        raisim_root, python_exe, env = find_raisim_runtime(project_root)

        worker_script = str(Path(__file__).resolve().parent / 'raisim_worker.py')
        cmd = [
            python_exe,
            worker_script,
            '--shared', str(self.shared_path),
            '--sock', self.sock_path,
            '--urdf', str(urdf_path),
            '--meta', str(meta_path),
            '--dt', str(self.dt),
            '--renderer', renderer,
            '--port', str(port),
        ]
        if raisim_root is not None:
            cmd.extend(['--raisim-root', str(raisim_root)])
        if viewer:
            cmd.append('--viewer')

        self.log_path = self.output_dir / 'raisim_worker.log'
        self._log_file = self.log_path.open('w')
        self.proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            self._listen_sock.settimeout(0.2)
            deadline = time.monotonic() + 15.0
            while True:
                try:
                    self._conn, _ = self._listen_sock.accept()
                    break
                except socket.timeout:
                    if self.proc.poll() is not None:
                        raise RuntimeError(f'worker exited with code {self.proc.returncode}')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('timed out waiting for worker socket')
            self._conn.settimeout(15.0)
        except Exception as exc:
            self.proc.poll()
            self._log_file.flush()
            detail = self.log_path.read_text(errors='replace').strip()
            self.close()
            raise RuntimeError(f'RaiSim worker failed to start: {exc}\n{detail}') from exc

    def _expect_ack(self):
        ack = self._conn.recv(1)
        if ack == b'X' or not ack:
            self.shared[SH_ALIVE] = 0.0
            return False
        if ack != b'K':
            raise RuntimeError(f'Unexpected RaiSim worker ack: {ack!r}')
        return True

    def reset(self, qpos, qvel):
        self.shared[SH_QPOS:SH_QPOS + 36] = np.asarray(qpos, dtype=np.float64)
        self.shared[SH_QVEL:SH_QVEL + 35] = np.asarray(qvel, dtype=np.float64)
        self.shared[SH_PUSH_ACTIVE] = 0.0
        self._conn.sendall(b'R')
        return self._expect_ack()

    def set_motor_targets(self, q_des, dq_des, kp, kd, tau_ff):
        self.shared[SH_QDES:SH_QDES + 29] = q_des
        self.shared[SH_DQDES:SH_DQDES + 29] = dq_des
        self.shared[SH_KP:SH_KP + 29] = kp
        self.shared[SH_KD:SH_KD + 29] = kd
        self.shared[SH_TAUFF:SH_TAUFF + 29] = tau_ff

    def set_push_force(self, force_vec, active):
        self.shared[SH_PUSH_VEC:SH_PUSH_VEC + 3] = force_vec
        self.shared[SH_PUSH_ACTIVE] = 1.0 if active else 0.0

    def send_hang_command(self, cmd):
        payload = cmd.encode('ascii')
        self._conn.sendall(b'H' + struct.pack('<H', len(payload)) + payload)
        return self._expect_ack()

    def update_hud(self, performance=None, jitter_monitor=None):
        perf = performance or {}
        fresh = time.time() - perf.get('wall_time', 0) < 2
        self.shared[SH_FSM_STATE] = {'STOP': 0, 'ZERO': 1, 'SDK_A': 2, 'DEXAMP': 3, 'FIX': 4}.get(
            perf.get('state'), -1
        ) if fresh else -1
        self.shared[SH_CTRL_HZ] = float(perf.get('control_hz', 0.0)) if fresh else 0.0
        self.shared[SH_FLT_ALPHA] = float(perf.get('filter_alpha', 0.0)) if fresh else 0.0
        if jitter_monitor is not None:
            self.shared[SH_JITTER_PCT] = min(100.0, float(jitter_monitor.jitter_score) * 100.0)
            self.shared[SH_TWITCH_PCT] = min(100.0, float(jitter_monitor.twitch_score) * 100.0)

    def step(self, n_steps=1):
        self._conn.sendall(b'S' + struct.pack('<I', int(n_steps)))
        return self._expect_ack()

    def poll_reset_request(self):
        cur = int(self.shared[SH_RESET_REQ])
        if cur != self._last_reset_req:
            self._last_reset_req = cur
            return True
        return False

    @property
    def qpos(self):
        return self.shared[SH_QPOS:SH_QPOS + 36]

    @property
    def qvel(self):
        return self.shared[SH_QVEL:SH_QVEL + 35]

    @property
    def tau_meas(self):
        return self.shared[SH_TAU:SH_TAU + 29]

    @property
    def pelvis_acc(self):
        return self.shared[SH_ACC:SH_ACC + 3]

    @property
    def sim_time(self):
        return float(self.shared[SH_TIME])

    @property
    def nonfoot_ground(self):
        return bool(self.shared[SH_NONFOOT] > 0.5)

    @property
    def max_penetration(self):
        return float(self.shared[SH_PENETRATION])

    @property
    def viewer_fps(self):
        return float(self.shared[SH_FPS])

    def is_paused(self):
        return bool(self.shared[SH_PAUSED] > 0.5)

    def suspension_state(self):
        return (
            bool(self.shared[SH_SUSP_EN] > 0.5),
            float(self.shared[SH_SUSP_Z]),
            float(self.shared[SH_SUSP_K]),
        )

    def is_running(self):
        return self.proc.poll() is None and bool(self.shared[SH_ALIVE] > 0.5)

    def close(self):
        if getattr(self, '_conn', None) is not None:
            try:
                self._conn.sendall(b'Q')
                self._conn.recv(1)
            except OSError:
                pass
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
        if getattr(self, '_listen_sock', None) is not None:
            try:
                self._listen_sock.close()
            except OSError:
                pass
            self._listen_sock = None
        Path(self.sock_path).unlink(missing_ok=True)
        if getattr(self, 'proc', None) is not None:
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        if getattr(self, '_log_file', None) is not None:
            try:
                self._log_file.close()
            except OSError:
                pass
            self._log_file = None
