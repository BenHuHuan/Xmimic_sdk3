"""Offline simulation suspension hotkey, HUD, terminal prompt, and physics check."""
import io
import os
import socket
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

import glfw
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sdk'))
sys.path.insert(0, str(ROOT / 'sdk' / 'policy' / 'sdk_a_to'))

from common.state_terminal import start_state_terminal
from render_bridge import HANG_KEYS, format_suspension_hud
from sim_suspension import SimSuspension

# 1. Verify viewer key bindings: Up/Down lift/lower, single-key release, no Left/Right conflict.
assert HANG_KEYS[glfw.KEY_UP] == 'UP'
assert HANG_KEYS[glfw.KEY_DOWN] == 'DOWN'
for k in (glfw.KEY_F10, glfw.KEY_DELETE, glfw.KEY_0):
    assert HANG_KEYS[k] == 'OFF', k
for k in (glfw.KEY_F9, glfw.KEY_INSERT, glfw.KEY_9):
    assert HANG_KEYS[k] == 'ON', k
for k in (glfw.KEY_KP_0, glfw.KEY_KP_5, glfw.KEY_H):
    assert HANG_KEYS[k] == 'TOGGLE', k
assert HANG_KEYS[glfw.KEY_EQUAL] == 'STRONGER' and HANG_KEYS[glfw.KEY_MINUS] == 'WEAKER'
assert glfw.KEY_LEFT not in HANG_KEYS and glfw.KEY_RIGHT not in HANG_KEYS

# 2. Verify SimSuspension lift/lower/release/stiffness-restore against the real MuJoCo model.
model = mujoco.MjModel.from_xml_path(str(ROOT / 'sdk/policy/sdk_a_to/scene_unitree_visual.xml'))
data = mujoco.MjData(model)
fixtures = np.load(ROOT / 'sdk/policy/sdk_a_to/fixtures.npz')
data.qpos[:] = fixtures['normal_p0_qpos']
data.qvel[:] = fixtures['normal_p0_qvel']
mujoco.mj_forward(model, data)

waist_id = model.body('waist_pitch_link').id
susp = SimSuspension(float(model.body_mass.sum()))
assert not susp.enabled
assert np.allclose(susp.force(data.xpos[waist_id], np.zeros(3)), 0.0)

susp.command(HANG_KEYS[glfw.KEY_F9], data.xpos[waist_id])
assert susp.enabled
z0 = float(susp.target[2])
assert z0 >= 1.15

susp.command(HANG_KEYS[glfw.KEY_UP], data.xpos[waist_id])
assert np.isclose(susp.target[2], z0 + 0.05)
f_up = susp.force(data.xpos[waist_id], np.zeros(3))
assert f_up[2] > susp.mass * 9.81, f_up

susp.command(HANG_KEYS[glfw.KEY_DOWN], data.xpos[waist_id])
assert np.isclose(susp.target[2], z0)

# Single-keypress release via F10 / 0 / Delete immediately disables suspension.
for release_key in (glfw.KEY_F10, glfw.KEY_0, glfw.KEY_DELETE):
    susp.command(HANG_KEYS[glfw.KEY_9], data.xpos[waist_id])
    assert susp.enabled
    susp.command(HANG_KEYS[release_key], data.xpos[waist_id])
    assert not susp.enabled
    assert np.allclose(susp.force(data.xpos[waist_id], np.zeros(3)), 0.0)

# Weakening to auto-off and then re-enabling via Up restores usable stiffness.
susp.command('ON', data.xpos[waist_id])
while susp.enabled:
    susp.command(HANG_KEYS[glfw.KEY_MINUS], data.xpos[waist_id])
assert not susp.enabled and susp.stiffness <= 65.0
susp.command(HANG_KEYS[glfw.KEY_UP], data.xpos[waist_id])
assert susp.enabled and susp.stiffness == 1000.0

# 3. Verify HUD text clearly communicates status, keys, and required window focus.
on_status, on_hint = format_suspension_hud(True, 1.25, 1000.0)
assert 'ENABLED' in on_status and '1.25m' in on_status and '1000N/m' in on_status
assert 'Up/Down' in on_hint and 'F10/0/Del' in on_hint
off_status, off_hint = format_suspension_hud(False, 1.25, 1000.0)
assert 'RELEASED' in off_status and 'OFF' in off_status
assert 'F9/9/Up' in off_hint and 'F10/0/Del' in off_hint

# 4. Verify UNIX datagram key socket round-trip used between viewer and physics loop.
with tempfile.TemporaryDirectory() as tmp:
    sock_path = os.path.join(tmp, 'viewer_keys.sock')
    rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    rx.bind(sock_path)
    rx.setblocking(False)
    tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    for k in (glfw.KEY_F9, glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_0):
        tx.sendto(HANG_KEYS[k].encode('ascii'), sock_path)
    received = [rx.recv(64).decode('ascii') for _ in range(4)]
    assert received == ['ON', 'UP', 'DOWN', 'OFF'], received
    tx.close()
    rx.close()

# 5. Verify simulation terminal startup prompt documents viewer focus and hotkeys.
class _DummySimCtrl:
    def __init__(self):
        self.calls = []

    def simulation_fix(self, value):
        self.calls.append(value)


ctrl = _DummySimCtrl()
out = io.StringIO()
with patch.dict(os.environ, {}, clear=True), \
        patch('sys.stdin', io.StringIO('HANG UP\nHANG DOWN\nHANG OFF\n')), \
        patch('sys.stdout', out):
    thread = start_state_terminal(ctrl)
    thread.join(timeout=2)
    assert not thread.is_alive()

text = out.getvalue()
assert 'HANG ON|OFF|UP|DOWN|STRONGER|WEAKER' in text
assert 'click MuJoCo window to focus' in text
assert 'F10/0/Del=release' in text
assert ctrl.calls == ['UP', 'DOWN', 'OFF'], ctrl.calls

print('PASS: suspension hotkeys (Up/Down/F10/0/Del), HUD prompts, terminal hints, and physics verified.')
