"""Offline verification for the RaiSim backend, URDF converter, IPC bridge, and CLI flags."""
import ctypes.util
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sdk' / 'policy' / 'sdk_a_to'))

from raisim_bridge import RaisimBridge, find_raisim_runtime
from raisim_urdf import mjcf_to_raisim_urdf, quat_wxyz_to_rpy
from raisim_worker import (
    SHARED_LEN,
    SH_ACC,
    SH_ALIVE,
    SH_DQDES,
    SH_FPS,
    SH_FSM_STATE,
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
    _SDLKeyPoller,
)
import run_chain

SCENE_XML = ROOT / 'sdk/policy/sdk_a_to/scene_unitree_visual.xml'
DEPLOY_JSON = ROOT / 'sdk/policy/sdk_a_to/deploy.json'
FIXTURES_NPZ = ROOT / 'sdk/policy/sdk_a_to/fixtures.npz'


def check_urdf_converter():
    contract = json.loads(DEPLOY_JSON.read_text())
    with tempfile.TemporaryDirectory() as tmp:
        out_urdf = Path(tmp) / 'tiangong3.urdf'
        meta = mjcf_to_raisim_urdf(SCENE_XML, out_urdf)
        assert out_urdf.is_file()
        assert meta['joint_names'] == contract['joint_names'], (
            len(meta['joint_names']),
            len(contract['joint_names']),
        )
        assert len(meta['joint_names']) == 29
        assert abs(meta['total_mass'] - 65.8178) < 0.05, meta['total_mass']

        tree = ET.parse(out_urdf)
        root = tree.getroot()
        links = root.findall('link')
        joints = root.findall('joint')
        assert len(links) == 39, len(links)
        assert len(joints) >= 29
        revolute = [j.attrib['name'] for j in joints if j.attrib.get('type') == 'revolute']
        assert revolute == contract['joint_names']

        mesh_dir = Path(meta['mesh_dir'])
        for mesh_el in root.findall('.//mesh'):
            fn = mesh_el.attrib['filename']
            assert (mesh_dir / fn).is_file(), f'Missing STL mesh: {mesh_dir / fn}'

        # Spot-check quat_wxyz_to_rpy consistency via ZYX Euler reconstruction
        for q in (
            [1.0, 0.0, 0.0, 0.0],
            [0.70710678, 0.70710678, 0.0, 0.0],
            [0.5, 0.5, -0.5, 0.5],
            [0.9238795, 0.0, 0.3826834, 0.0],
        ):
            r, p, y = quat_wxyz_to_rpy(q)
            cr, sr = np.cos(r / 2), np.sin(r / 2)
            cp, sp = np.cos(p / 2), np.sin(p / 2)
            cy, sy = np.cos(y / 2), np.sin(y / 2)
            q_rec = np.array([
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ])
            qn = np.asarray(q) / np.linalg.norm(q)
            assert abs(abs(float(np.dot(q_rec, qn))) - 1.0) < 1e-6, (q, q_rec)
    print('PASS: urdf converter & mesh/joint contract')


def check_qrot_equivalence():
    rng = np.random.default_rng(7)
    for _ in range(25):
        q_raw = rng.normal(size=4) * rng.uniform(0.1, 3.0)
        q_unit = q_raw / np.linalg.norm(q_raw)
        r_mj = np.empty(9)
        mujoco.mju_quat2Mat(r_mj, q_unit)
        assert np.allclose(run_chain.qrot(q_raw), r_mj.reshape(3, 3), atol=1e-12)
        assert np.allclose(run_chain.qrot(q_unit), r_mj.reshape(3, 3), atol=1e-12)
    assert np.allclose(run_chain.qrot([0, 0, 0, 0]), np.eye(3))
    print('PASS: pure-NumPy qrot matches MuJoCo mju_quat2Mat')


def check_sdl_scancodes():
    if not ctypes.util.find_library('SDL2'):
        print('SKIP: SDL2 library not present for scancode lookup')
        return
    poller = _SDLKeyPoller()
    if poller._sdl is None:
        print('SKIP: SDL2 cdll did not initialize scancode map')
        return
    expected = {'ON', 'OFF', 'TOGGLE', 'UP', 'DOWN', 'STRONGER', 'WEAKER', 'PAUSE_TOGGLE', 'RESET'}
    assert expected.issubset(set(poller._bindings.values())), set(poller._bindings.values())
    print(f'PASS: SDL2 scancodes resolved ({len(poller._bindings)} bindings)')


def check_runtime_discovery():
    with tempfile.TemporaryDirectory() as tmp:
        fake_root = Path(tmp) / 'raisim2Lib'
        (fake_root / 'raisim' / 'lib').mkdir(parents=True)
        (fake_root / 'rayrai' / 'lib').mkdir(parents=True)
        (fake_root / 'build-py' / 'raisimPy').mkdir(parents=True)
        py = fake_root / '.venv' / 'bin' / 'python'
        py.parent.mkdir(parents=True)
        py.write_text('#!/bin/sh\nexit 0\n')
        py.chmod(0o755)
        with patch.dict(
            os.environ,
            {
                'RAISIM2LIB_ROOT': str(fake_root),
                'PYTHONPATH': '/opt/ros/humble/lib/python3.10/site-packages:/custom/path',
            },
        ):
            rroot, py_exe, env = find_raisim_runtime(ROOT)
            assert rroot == fake_root.resolve()
            assert Path(py_exe) == py
            assert '/python3.10/' not in env['PYTHONPATH']
            assert '/custom/path' in env['PYTHONPATH']
            assert str(fake_root.resolve() / 'build-py' / 'raisimPy') in env['PYTHONPATH']
    print('PASS: RaiSim runtime discovery & Python ABI env isolation')


def check_bridge_ipc_roundtrip():
    contract = json.loads(DEPLOY_JSON.read_text())
    fixtures = np.load(FIXTURES_NPZ)
    q0 = fixtures['normal_p0_qpos']
    v0 = fixtures['normal_p0_qvel']

    mock_script = '''
import argparse, socket, struct
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--shared', required=True)
ap.add_argument('--sock', required=True)
ap.add_argument('--dt', type=float, default=0.001)
ns, _ = ap.parse_known_args()
a = np.load(ns.shared, mmap_mode='r+')
a[0] = 1.0  # SH_ALIVE
a[7] = 1000.0  # SH_SUSP_K
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(ns.sock)
while True:
    op = sock.recv(1)
    if not op or op == b'Q':
        if op == b'Q':
            sock.sendall(b'K')
        break
    elif op == b'R':
        a[2] = 0.0
        a[20:23] = [0.1, -0.2, 9.81]
        a[8] = 0.0
        a[9] = 0.001
        sock.sendall(b'K')
    elif op == b'H':
        n = struct.unpack('<H', sock.recv(2))[0]
        cmd = sock.recv(n).decode('ascii')
        if cmd == 'ON':
            a[5] = 1.0
            a[6] = 1.15
        elif cmd == 'UP':
            a[5] = 1.0
            a[6] += 0.05
        elif cmd == 'OFF':
            a[5] = 0.0
        sock.sendall(b'K')
    elif op == b'S':
        n = struct.unpack('<I', sock.recv(4))[0]
        a[2] += 0.001 * n
        a[103:132] = a[248:277] + a[190:219] * (a[132:161] - a[39:68])
        a[3] = 60.0
        sock.sendall(b'K')
a[0] = 0.0
'''
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        fake_worker = tmp_path / 'mock_worker.py'
        fake_worker.write_text(mock_script)
        # Monkeypatch raisim_bridge's Path(__file__).parent/'raisim_worker.py' via subprocess arg?
        # Easier: write a wrapper raisim2Lib/.venv/bin/python that ignores original worker path and runs mock_worker.py!
        fake_root = tmp_path / 'raisim2Lib'
        (fake_root / 'raisim' / 'lib').mkdir(parents=True)
        (fake_root / 'rayrai' / 'lib').mkdir(parents=True)
        py = fake_root / '.venv' / 'bin' / 'python'
        py.parent.mkdir(parents=True)
        py.write_text(f'#!/bin/sh\nshift\nexec {sys.executable!r} {str(fake_worker)!r} "$@"\n')
        py.chmod(0o755)

        out_dir = tmp_path / 'run_out'
        with patch.dict(os.environ, {'RAISIM2LIB_ROOT': str(fake_root), 'RAISIM_PYTHON': str(py)}):
            bridge = RaisimBridge(SCENE_XML, contract, out_dir, dt=0.001, viewer=True, renderer='rayrai', project_root=ROOT)
            try:
                assert bridge.is_running()
                assert bridge.reset(q0, v0)
                assert np.allclose(bridge.qpos, q0)
                assert np.allclose(bridge.qvel, v0)
                assert np.allclose(bridge.pelvis_acc, [0.1, -0.2, 9.81])

                q_des = q0[7:] + 0.1
                kp = np.full(29, 20.0)
                bridge.set_motor_targets(q_des, np.zeros(29), kp, np.zeros(29), np.full(29, 1.5))
                bridge.set_push_force((10.0, 0.0, 0.0), True)
                assert abs(bridge.shared[SH_PUSH_ACTIVE] - 1.0) < 1e-9
                assert bridge.step(2)
                assert abs(bridge.sim_time - 0.002) < 1e-9
                assert np.allclose(bridge.tau_meas, 1.5 + 20.0 * 0.1)
                assert abs(bridge.viewer_fps - 60.0) < 1e-9

                assert bridge.send_hang_command('ON')
                assert bridge.send_hang_command('UP')
                assert bridge.suspension_state() == (True, 1.2, 1000.0)
                assert bridge.send_hang_command('OFF')
                assert bridge.suspension_state()[0] is False

                bridge.update_hud({'wall_time': 1e12, 'state': 'SDK_A', 'control_hz': 100.0, 'filter_alpha': 0.5})
                assert int(bridge.shared[SH_FSM_STATE]) == 2
            finally:
                bridge.close()
    print('PASS: RaisimBridge UNIX socket + mmap IPC roundtrip')


def check_cli_dry_run():
    for renderer in ('rayrai', 'tcp'):
        res = subprocess.run(
            [
                sys.executable,
                str(ROOT / 'cli.py'),
                'run',
                '--interface', 'lo',
                '--physics', 'raisim',
                '--raisim-renderer', renderer,
                '--headless',
                '--seconds', '2',
                '--dry-run',
            ],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=True,
        )
        assert f'RaiSim + {renderer}' in res.stdout, res.stdout
        assert f'--physics raisim --raisim-renderer {renderer}' in res.stdout, res.stdout
    print('PASS: cli.py --physics raisim --dry-run')


if __name__ == '__main__':
    check_urdf_converter()
    check_qrot_equivalence()
    check_sdl_scancodes()
    check_runtime_discovery()
    check_bridge_ipc_roundtrip()
    check_cli_dry_run()
    print('ALL OFFLINE RAISIM BACKEND CHECKS PASSED.')
