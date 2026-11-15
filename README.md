# Xrobot3 (Xmimic SDK3)

[中文文档 (Chinese)](README_zh.md)

**Unified control, sim-to-real validation, and hardware-safety runtime for Tiangong 3 (29-DoF).**

One SDK controller, 5-state FSM, PD torque limiter, and ONNX policy pipeline shared across **local MuJoCo simulation**, **wired offboard control**, and **onboard real-robot deployment** over ROS 2 / FastRTPS DDS.

---

## Key Features

- **MuJoCo 4.0 Default Simulation (`--physics 4.0`)**: Runs physics at 1 kHz with MuJoCo 4.0 by default (`.venv-mj4`), with instant fallback to `--physics 3.3.7` (`.venv`).
- **Real-Time Fatal Hardware-Damage Jitter & Shock Monitor**:
  - **50 Hz ZOH-immune** signal processing separates normal 1–4 Hz locomotion and 20 ms policy steps from destructive hardware resonance.
  - Covers **10–25 Hz** sustained $I^2t$ thermal / gearbox alternating fatigue, **30–250 Hz** intra-window PD/contact hunting, **ISO 20816-inspired** $\ge 10\text{ Hz}$ velocity vibration, and **$k_p$-scaled** command/torque shocks.
  - Highlights offending joints directly in the 3D viewer (`amber` warning $\ge 55\%$, `red` pulsing fatal risk $\ge 75\%$) alongside a fixed `0–105%` real-time oscilloscope and HUD banner.
- **Unified 5-State FSM (`100 Hz` Control / `50 Hz` Policy)**:
  - `STOP`, `ZERO`, `FIX` (2 s interpolated standing hold), `SDK-A` (tracking / recovery ONNX policy with `FILTER` & `REF` slicing), and `WALK` (`DEXAMP` velocity locomotion via `VEL vx vy [wz]`).
- **Simulation-Aligned PD Torque Limiter**: Enforces `deploy.json` effort bounds across 29 serial joints, parallel ankle motor pairs (`sptlib`), and `float32` wire serialization (`sdk/common/torque_limiter.py`).
- **One-Command Remote Deploy & Driver Control**: Built-in SSH/rsync deployment (`deploy`), onboard `body_control` start/stop (`body`), DDS adapter discovery (`interfaces`), and contract verification (`check`).
- **Dual ROS 2 Support**: Auto-detects ROS 2 **Humble** (Ubuntu 22.04, Python 3.10) and **Jazzy** (Ubuntu 24.04, Python 3.12).

---

## Architecture

```text
  Terminal / WebSocket Console (port 12888)
                 │
                 ▼
  SDK Controller (100 Hz) + ONNX Policy (50 Hz) + Torque Limiter
                 │  ROS 2 / FastRTPS DDS (29-DoF CmdMotorCtrl & MotorStatusMsg)
        ┌────────┴────────┐
        ▼                 ▼
  MuJoCo 4.0 Sim     Real Tiangong 3
  (1 kHz + 3D HUD    (Onboard / Wired Offboard
   Jitter Monitor)    body_control)
```

---

## Quick Start

### 1. Install (Ubuntu 22.04 Humble / 24.04 Jazzy, `x86_64`)

Install ROS 2 first, then:

```bash
git clone -b dev https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
cd Xrobot3
bash install.sh
./xrobot3 check
```

To set up the MuJoCo 4.0 simulation environment (`.venv-mj4`):

```bash
bash scripts/setup_mujoco4_dev.sh
```

### 2. Run Simulation

```bash
# Interactive 3D simulation (MuJoCo 4.0 default)
./xrobot3 run --interface lo

# Switch back to MuJoCo 3.3.7 if needed
./xrobot3 run --interface lo --physics 3.3.7

# Headless smoke test (5 seconds)
./xrobot3 run --interface lo --headless --seconds 5
```

### 3. Run on Real Robot

```bash
# List available network interfaces for DDS
./xrobot3 interfaces

# Wired offboard control (replace ROBOT_INTERFACE with adapter name)
./xrobot3 run --interface ROBOT_INTERFACE

# Deploy, start onboard body_control, and launch over SSH
./xrobot3 deploy --host USER@ROBOT
./xrobot3 body --host USER@ROBOT
./xrobot3 run --host USER@ROBOT

# Stop onboard body_control when finished
./xrobot3 body --host USER@ROBOT --stop
```

> Simulation defaults to DDS domain `174` (`lo`); real robot defaults to domain `0` (default peer `192.168.41.1`, override via `--robot-ip`). Use `--dry-run` to inspect any launch command without executing.

---

## Interactive Terminal Commands

Case-insensitive commands accepted in both simulation and onboard/offboard terminal:

| Command | Description |
| :--- | :--- |
| `STOP` | Hold using SDK STOP controller (not zero torque) |
| `ZERO` | Move to configured ready posture |
| `FIX` / `STAND` | Interpolate to fixed standing posture in 2 s and hold (PD only, no ONNX) |
| `SDK-A` | Run SDK-A Tracking / Recovery ONNX policy |
| `FILTER [arg]` | Set SDK-A EMA action smoothing (`ON` / `OFF` / `LIGHT` / `MED` / `STRONG` / `<alpha>`) |
| `REF [arg]` | Select SDK-A reference slice (`STAND` / `DANCE` / `FULL` / `<start> <end>`) |
| `WALK` | Run `DEXAMP` velocity locomotion policy at zero velocity |
| `VEL vx vy [wz]` | Update walking velocity (`m/s`, `rad/s`) while `WALK` is active |
| `HANG ON\|OFF\|UP\|DOWN\|STRONGER\|WEAKER` | *(Simulation only)* Virtual elastic torso suspension (hotkeys: `F9`/`F10`, `7`/`8`, `Up`/`Down`) |

---

## CLI Reference

| Command | Purpose |
| :--- | :--- |
| `./xrobot3 run` | Launch local simulation, wired offboard, or `--host` SSH onboard control |
| `./xrobot3 deploy --host USER@ROBOT` | Sync workspace via `rsync` (with progress/ETA) and run `install.sh` remotely |
| `./xrobot3 body --host USER@ROBOT [--stop]` | Start or stop onboard `body_control` driver |
| `./xrobot3 interfaces` | List local IPv4 interfaces for DDS selection |
| `./xrobot3 check` | Verify Python env, ROS 2 messages, and ONNX / `deploy.json` contracts |

---

## Project Structure

```text
Xrobot3/
├── xrobot3                     # Main CLI wrapper
├── cli.py                      # Unified CLI entrypoint (run, deploy, body, check, interfaces)
├── network.py                  # Interface discovery & FastRTPS DDS profile generation
├── env.sh / install.sh         # Auto-detecting ROS 2 Humble/Jazzy environment setup
├── scripts/
│   ├── setup_mujoco4_dev.sh    # MuJoCo 4.0 (.venv-mj4) installer
│   └── check_onboard_terminal.py
├── sdk/
│   ├── rl_control_node*.py     # 100 Hz SDK control nodes (hardware & sim)
│   ├── FSM/                    # 5-state FSM controller
│   ├── common/                 # Torque limiter, robot interface, terminal & websocket console
│   ├── config/                 # Hardware & simulation YAML configs
│   └── policy/
│       ├── sdk_a_to/           # SDK-A ONNX, deploy.json contract, JitterMonitor & RenderBridge
│       ├── dex_amp/            # DEXAMP velocity locomotion policy
│       └── fix/                # Fixed standing posture targets & PD gains
├── xmujoco/                    # MuJoCo simulator view & async bridge
├── assets/                     # Robot scenes & visual assets
└── docs/                       # Engineering & handoff notes
```

---

## Verification & Safety Notes

- Offline FSM & terminal transition check:
  ```bash
  source env.sh && python scripts/check_onboard_terminal.py
  ```
- Runtime logs and per-run summaries (including `"jitter"` metrics) are saved under `output/`.
- **Simulation testing is not hardware validation.** Always verify joint feedback frames, DDS domain, network interface, and actuator limits (`sdk/policy/sdk_a_to/deploy.json`) before running on physical hardware.
- Third-party upstream sources and pinned commits are documented in [UPSTREAM.json](UPSTREAM.json).
