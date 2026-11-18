# Xrobot3 (Xmimic SDK3)

[中文文档 (Chinese)](README_zh.md)

**Unified control, sim-to-real validation, and hardware-safety runtime for Tiangong 3 (29-DoF).**

One SDK controller, 5-state FSM, PD torque limiter, and ONNX policy pipeline shared across **local MuJoCo simulation**, **wired offboard control**, and **onboard real-robot deployment** over ROS 2 / FastRTPS DDS.

---

## Key Features

- **MuJoCo Simulation (`--physics mujoco`, default `4.0`) + RaiSim Backend (`--physics raisim`)**: Runs MuJoCo physics at 1 kHz by default (`--mujoco-version 4.0` via `.venv-mj4`, fallback `--mujoco-version 3.3.7` via `.venv`), or switches to `--physics raisim` (**RaiSim + Rayrai** 3D renderer or `--raisim-renderer tcp` `RaisimServer`). Legacy `--physics 4.0` / `3.3.7` aliases remain supported.
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
# Interactive 3D simulation (MuJoCo 4.0 default: --physics mujoco --mujoco-version 4.0)
./xrobot3 run --interface lo

# RaiSim physics + Rayrai 3D renderer (requires raisim2Lib + activation.raisim)
./xrobot3 run --interface lo --physics raisim

# RaiSim headless smoke test or RaisimServer TCP visualizer
./xrobot3 run --interface lo --physics raisim --headless --seconds 5
./xrobot3 run --interface lo --physics raisim --raisim-renderer tcp

# Switch back to MuJoCo 3.3.7 if needed
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7

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
| `HANG ON\|OFF\|UP\|DOWN\|STRONGER\|WEAKER` | *(Simulation only)* Virtual elastic torso suspension (see [Simulation Suspension Hotkeys](#simulation-suspension-hotkeys)) |

### Simulation Suspension Hotkeys

> **Window focus required:** Viewer keyboard shortcuts only take effect when the **MuJoCo 3D viewer window** has keyboard focus (click inside the viewer window first). When the terminal has focus, use `HANG ON|OFF|UP|DOWN|STRONGER|WEAKER` instead.

| Action | MuJoCo Viewer Hotkeys | Notes |
| :--- | :--- | :--- |
| Enable suspension | `F9` / `9` / `Insert` (or `↑` when released) | Engages virtual elastic support (`ENABLED`) |
| Lift / lower robot | `↑` / `↓` (`PageUp`/`PageDown`, `7`/`8`, `KP7`/`KP9` & `KP1`/`KP3`) | Raises or lowers target height by `0.05 m` per press |
| One-key release | `F10` / `0` / `Delete` | Immediately disables suspension support (`RELEASED (OFF)`) |
| Toggle enable / release | `H` / `KP0` / `KP5` | Switches between `ENABLED` and `RELEASED (OFF)` |
| Increase / decrease stiffness | `.` / `KP+` (`KP8`/`KP6`) and `,` / `KP-` (`KP2`/`KP4`) | Adjusts spring stiffness (`N/m`); decreasing below `65 N/m` releases (`-`/`=` remain free for MuJoCo sim speed) |

- **Viewer HUD status:** The top-left overlay always shows `Suspension: ENABLED (z=...m, k=...N/m)` or `RELEASED (OFF)`, along with the `Sling keys (focus window)` reminder.

---

## CLI Reference

| Command | Purpose |
| :--- | :--- |
| `./xrobot3 run` | Launch local simulation, wired offboard, or `--host` SSH onboard control |
| `./xrobot3 deploy --host USER@ROBOT` | Sync workspace via `rsync` (with progress/ETA) and run `install.sh` remotely |
| `./xrobot3 body --host USER@ROBOT [--stop]` | Start or stop onboard `body_control` driver |
| `./xrobot3 interfaces` | List local IPv4 interfaces for DDS selection |
| `./xrobot3 sync` | Auto-update SDK files to the latest `main` release with progress bar (works with or without `.git`; `--reinstall` / `--tarball` / `--dry-run`) |
| `./xrobot3 check` | Verify Python env, ROS 2 messages, and ONNX / `deploy.json` contracts |
| `./xrobot3 diagnose` | Inspect ROS 2, GPU, DDS network, default MuJoCo, and optional RaiSim readiness (`--json`, `--strict-raisim`) |

---

## Optional: Installing RaiSim + Rayrai (`--physics raisim`)

> **Decoupled by design:** RaiSim is completely **optional**. If RaiSim or a license key is not installed, default `./xrobot3 run --interface lo` (`--physics mujoco`), `./xrobot3 check`, `./xrobot3 diagnose`, and real-robot deployment work with zero RaiSim dependencies. Running `--physics raisim` without RaiSim/license exits immediately with an actionable prompt and suggests `--physics mujoco`.

1. **Install system dependencies** (Ubuntu 22.04 / 24.04):
   ```bash
   sudo apt-get update && sudo apt-get install -y cmake g++ libeigen3-dev libsdl2-dev python3-venv
   ```
2. **Clone `raisim2Lib` and build `raisimPy` + `rayrai`** (inside its own Python venv, e.g. `.venv`, so it does not conflict with ROS 2 Humble Python 3.10):
   ```bash
   git clone https://github.com/raisimTech/raisim2Lib.git ../raisim2Lib
   cd ../raisim2Lib
   python3 -m venv .venv && source .venv/bin/activate
   pip install numpy
   source ./raisim_env.sh
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
   cmake --build build --parallel
   ```
   > Auto-discovery checks `$RAISIM2LIB_ROOT`, sibling `../raisim2Lib`, and `~/raisim2Lib`. Set `export RAISIM2LIB_ROOT=/path/to/raisim2Lib` (and optionally `RAISIM_PYTHON`) if placed elsewhere.
3. **Install your RaiSim activation key (`activation.raisim`)**:
   - Request a free Academic or Trial key from the RaiSim License Form (`https://forms.gle/4bcS2GByKYdeKxmn7` / RaiSim docs).
   - Place the issued key file at `~/.raisim/activation.raisim` (or point `RAISIM_ACTIVATION_KEY=/path/to/activation.raisim`).
4. **Verify readiness before running**:
   ```bash
   ./xrobot3 diagnose
   ./xrobot3 run --interface lo --physics raisim
   ```

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
│       ├── sdk_a_to/           # SDK-A ONNX, deploy.json contract, JitterMonitor, RenderBridge & RaiSim bridge/worker/URDF
│       ├── dex_amp/            # DEXAMP velocity locomotion policy
│       └── fix/                # Fixed standing posture targets & PD gains
├── xmujoco/                    # MuJoCo simulator view & async bridge
├── assets/                     # Robot scenes & visual assets
└── docs/                       # Engineering & handoff notes
```

---

## Verification & Safety Notes

Recommended pre-release verification suite:

```bash
# 1. Environment, 29-DoF ONNX/deploy contract, MuJoCo 4.0.0 & backend diagnostics
./xrobot3 check
./xrobot3 diagnose

# 2. Offline 25-transition FSM, WALK/VEL, FILTER/REF, HANG-exclusion & RaiSim backend checks
source env.sh && python scripts/check_onboard_terminal.py
.venv-mj4/bin/python scripts/check_sim_suspension.py
.venv-mj4/bin/python scripts/check_raisim_backend.py

# 3. SDK auto-updater & MuJoCo 4.0 / 3.3.7 simulation smoke tests
./xrobot3 sync
./xrobot3 run --interface lo --physics mujoco --headless --seconds 5
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7 --headless --seconds 4
```

- Runtime logs and per-run summaries (including `"jitter"` metrics) are saved under `output/`.
- **Simulation testing is not hardware validation.** Always verify joint feedback frames, DDS domain, network interface, and actuator limits (`sdk/policy/sdk_a_to/deploy.json`) before running on physical hardware.
- Third-party upstream sources and pinned commits are documented in [UPSTREAM.json](UPSTREAM.json).
