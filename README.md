# Xrobot3 (Xmimic SDK3)

[中文文档 (Chinese)](README_zh.md)

**Unified control, sim-to-real validation, and hardware-safety runtime for Tiangong 3 (29-DoF).**

One SDK controller, 5-state FSM, PD torque limiter, and ONNX policy pipeline shared across **local 1 kHz simulation** (MuJoCo default / optional RaiSim), **wired offboard control**, and **onboard real-robot deployment** over ROS 2 / FastRTPS DDS.

---

## Key Features

- **Pluggable 1 kHz Simulation Backends (`--physics mujoco` | `--physics raisim`)**:
  - **MuJoCo Default (`--physics mujoco`)**: Runs MuJoCo `4.0` by default (`--mujoco-version 4.0` via `.venv-mj4`) with instant fallback to `--mujoco-version 3.3.7` (`.venv`). Legacy `--physics 4.0` / `3.3.7` aliases remain supported.
  - **Optional RaiSim + Rayrai (`--physics raisim`)**: Fully decoupled optional backend with **Rayrai** 3D renderer (`--raisim-renderer rayrai`) or `RaisimServer` (`--raisim-renderer tcp`). Absence of RaiSim never affects default MuJoCo or hardware workflows.
- **Real-Time Fatal Hardware-Damage Jitter & Shock Monitor (`JitterMonitor v3`)**:
  - **50 Hz ZOH-immune** signal processing separates normal 1–4 Hz locomotion and 20 ms policy steps from destructive hardware resonance.
  - Covers **10–25 Hz** sustained $I^2t$ thermal / gearbox alternating fatigue, **30–250 Hz** intra-window PD/contact hunting, **ISO 20816-inspired** $\ge 10\text{ Hz}$ velocity vibration, and **$k_p$-scaled** command/torque shocks.
  - Highlights offending joints directly in the MuJoCo 3D viewer (`amber` warning $\ge 55\%$, `red` pulsing fatal risk $\ge 75\%$) alongside a fixed `0–105%` real-time oscilloscope and HUD banner.
- **Unified 5-State FSM (`50 Hz` Control & Policy / `1 kHz` Low-Level PD)**:
  - Starts in safe `STOP` hold; transitions cleanly across `STOP`, `ZERO`, `FIX` (2 s smooth interpolated standing), `SDK-A` (starts on safe `STAND` `50..240` slice; supports `FILTER` & `REF`), and `WALK` (`DEXAMP` velocity locomotion via `VEL vx vy [wz]`).
- **Simulation-Aligned PD Torque Limiter**: Enforces `deploy.json` effort bounds across 29 serial joints, parallel ankle motor pairs (`sptlib`), and `float32` wire serialization (`sdk/common/torque_limiter.py`).
- **One-Command Deploy, Sync & Diagnostics**: Built-in SSH/rsync deployment (`deploy`, `upload`, `remote`), onboard `body_control` start/stop (`body`), GitHub release auto-updater (`sync`), contract verification (`check`), and full environment diagnostics (`diagnose`).
- **Dual ROS 2 Support**: Auto-detects ROS 2 **Humble** (Ubuntu 22.04, Python 3.10) and **Jazzy** (Ubuntu 24.04, Python 3.12).

---

## Architecture

```text
          Terminal CLI / WebSocket Console (port 12888)
                               │
                               ▼
    SDK Controller & 5-State FSM (50 Hz) + ONNX Policy (50 Hz)
                     + PD Torque Limiter
                               │  ROS 2 / FastRTPS DDS
                               │  (29-DoF CmdMotorCtrl & MotorStatusMsg)
                 ┌─────────────┴─────────────┐
                 ▼                           ▼
         1 kHz Local Sim              Real Tiangong 3
   • MuJoCo 4.0 / 3.3.7 (default,    • Onboard or Wired Offboard
     3D HUD + Jitter Monitor)          body_control (1 kHz PD)
   • RaiSim + Rayrai (optional)
```

---

## Quick Start

### 1. Install (Ubuntu 22.04 Humble / 24.04 Jazzy, `x86_64`)

Install your matching ROS 2 desktop/base distribution first, then:

```bash
git clone https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
cd Xrobot3
bash install.sh
bash scripts/setup_mujoco4_dev.sh   # Builds default MuJoCo 4.0 (.venv-mj4); skip if using --mujoco-version 3.3.7
./xrobot3 check
./xrobot3 diagnose
```

### 2. Run Simulation

```bash
# Interactive 3D simulation (default: --physics mujoco --mujoco-version 4.0)
./xrobot3 run --interface lo

# Headless smoke test (5 seconds)
./xrobot3 run --interface lo --headless --seconds 5

# Fallback to MuJoCo 3.3.7 (.venv) if .venv-mj4 is not installed
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7

# Optional: RaiSim physics + Rayrai 3D renderer (see setup guide below)
./xrobot3 run --interface lo --physics raisim
./xrobot3 run --interface lo --physics raisim --raisim-renderer tcp
./xrobot3 run --interface lo --physics raisim --headless --seconds 5
```

### 3. Run on Real Robot

```bash
# List available local IPv4 interfaces for DDS
./xrobot3 interfaces

# Option A: Wired offboard control (replace ROBOT_INTERFACE with your NIC name)
./xrobot3 run --interface ROBOT_INTERFACE

# Option B: Deploy to onboard PC, start body_control driver, and run over SSH
./xrobot3 deploy --host USER@ROBOT
./xrobot3 body --host USER@ROBOT
./xrobot3 run --host USER@ROBOT

# Stop onboard body_control when finished
./xrobot3 body --host USER@ROBOT --stop
```

> Simulation defaults to DDS domain `174` (`lo`); real robot defaults to domain `0` (default peer `192.168.41.1`, override via `--robot-ip` / `--domain`). Append `--dry-run` to preview any local or SSH command without executing it.

---

## Interactive Terminal Commands

Case-insensitive commands accepted in both simulation and onboard/offboard terminal:

| Command | Description |
| :--- | :--- |
| `STOP` | Hold current posture using SDK `STOP` PD controller (startup default; non-zero torque) |
| `ZERO` | Move smoothly to configured zero/ready posture |
| `FIX` / `STAND` | Interpolate to fixed standing posture in 2 s and hold (PD only, no ONNX) |
| `SDK-A` | Run SDK-A Tracking / Recovery ONNX policy (defaults to safe `STAND` `50..240` slice) |
| `FILTER [arg]` | Configure SDK-A EMA action smoothing (`ON` / `OFF` / `LIGHT` / `MED` / `STRONG` / `<alpha>`) |
| `REF [arg]` | Select SDK-A reference slice (`STAND` / `DANCE` / `FULL` / `<start> <end>`) |
| `WALK` | Run `DEXAMP` velocity locomotion policy (starts at zero velocity) |
| `VEL vx vy [wz]` | Update target velocity (`m/s`, `rad/s`) while `WALK` is active |
| `HANG ON\|OFF\|UP\|DOWN\|STRONGER\|WEAKER` | *(Simulation only)* Virtual elastic torso suspension (see [Simulation Suspension Hotkeys](#simulation-suspension-hotkeys)) |

### Simulation Suspension Hotkeys

> **Window focus required:** Keyboard shortcuts only take effect when the **active 3D viewer window** (MuJoCo viewer or RaiSim Rayrai window) has keyboard focus — click inside the 3D window first. When the terminal has focus, type `HANG ON|OFF|UP|DOWN|STRONGER|WEAKER`.

| Action | 3D Viewer Hotkeys (MuJoCo & Rayrai) | Notes |
| :--- | :--- | :--- |
| Enable suspension | `F9` / `9` / `Insert` (or `↑` when released) | Engages virtual elastic support (`ENABLED`) |
| Lift / lower robot | `↑` / `↓` (`PageUp`/`PageDown`, `7`/`8`, `KP7`/`KP9` & `KP1`/`KP3`) | Adjusts target suspension height by `±0.05 m` per press |
| One-key release | `F10` / `0` / `Delete` | Immediately disables suspension support (`RELEASED (OFF)`) |
| Toggle enable / release | `H` / `KP0` / `KP5` | Toggles between `ENABLED` and `RELEASED (OFF)` |
| Increase / decrease stiffness | `.` / `KP+` (`KP8`/`KP6`) and `,` / `KP-` (`KP2`/`KP4`) | Adjusts spring stiffness (`N/m`); dropping below `65 N/m` auto-releases (`-`/`=` remain free for MuJoCo sim speed) |

- **Status feedback:** MuJoCo's top-left HUD overlay displays `Suspension: ENABLED (z=...m, k=...N/m)` or `RELEASED (OFF)` with key hints; Rayrai displays live sling status in the window title, renders a 3D green tether, and provides interactive ImGui sliders (plus `Space` to pause and `R` to reset).

---

## CLI Reference

| Command | Purpose |
| :--- | :--- |
| `./xrobot3 run` | Launch local simulation (`--interface lo`), wired offboard (`--interface NIC`), or SSH onboard (`--host USER@ROBOT`) |
| `./xrobot3 check` | Verify Python environment, ROS 2 messages, torque limiter, and 29-DoF ONNX / `deploy.json` / URDF contracts |
| `./xrobot3 diagnose` | Inspect ROS 2, GPU power/thermals, DDS NICs, default MuJoCo, and optional RaiSim readiness (`--json`, `--strict-raisim`) |
| `./xrobot3 interfaces` | List local IPv4 network interfaces for FastRTPS DDS transport |
| `./xrobot3 sync` | Update SDK files from `main` with progress bar (git or tarball archive; `--reinstall`, `--tarball`, `--dry-run`) |
| `./xrobot3 deploy --host USER@ROBOT` | Sync workspace over `rsync` (with progress/ETA) and run `install.sh` on the robot |
| `./xrobot3 upload --host USER@ROBOT` | Fast code-only `rsync` to the robot without re-running `install.sh` |
| `./xrobot3 body --host USER@ROBOT [--stop]` | Start or stop the onboard `body_control` real-time ROS 2 driver |
| `./xrobot3 remote --host USER@ROBOT` | Start onboard SDK control session over SSH |

Key `./xrobot3 run` simulation flags: `--physics {mujoco,raisim}` (default `mujoco`), `--mujoco-version {4.0,3.3.7}` (default `4.0`), `--raisim-renderer {rayrai,tcp}` (default `rayrai`), `--headless`, `--seconds N`, `--domain ID`, `--dry-run`.

---

## Optional: Installing RaiSim + Rayrai (`--physics raisim`)

> **Decoupled by design:** RaiSim is strictly **optional**. Without `raisim2Lib` or an activation key, `./xrobot3 run --interface lo` (`--physics mujoco`), `./xrobot3 check`, `./xrobot3 diagnose`, and hardware control all operate normally with zero RaiSim dependencies. Invoking `--physics raisim` when unconfigured exits immediately with setup instructions and a `--physics mujoco` fallback command.

1. **Install build dependencies** (Ubuntu 22.04 / 24.04):
   ```bash
   sudo apt-get update && sudo apt-get install -y cmake g++ libeigen3-dev libsdl2-dev python3-venv
   ```
2. **Clone `raisim2Lib` and build `raisimPy` + `rayrai`** (in its own `.venv` so Python 3.12 bindings stay isolated from ROS 2 Humble Python 3.10):
   ```bash
   git clone https://github.com/BenHuHuan/raisim2Lib.git ../raisim2Lib
   cd ../raisim2Lib
   python3 -m venv .venv && source .venv/bin/activate
   pip install numpy
   source ./raisim_env.sh
   cmake -S . -B build-py -DCMAKE_BUILD_TYPE=Release -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
   cmake --build build-py --parallel
   ```
   > Auto-discovery checks `$RAISIM2LIB_ROOT`, sibling `../raisim2Lib`, and `~/raisim2Lib` (supporting both `build-py/raisimPy` and `build/raisimPy`, as well as upstream `raisimTech/raisim2Lib`). Set `export RAISIM2LIB_ROOT=/path/to/raisim2Lib` if installed elsewhere.
3. **Install your RaiSim activation key (`activation.raisim`)**:
   - Request a free Academic or Trial key from the [RaiSim License Request Form](https://forms.gle/4bcS2GByKYdeKxmn7).
   - Save the file to `~/.raisim/activation.raisim` (or set `export RAISIM_ACTIVATION_KEY=/path/to/activation.raisim`).
4. **Verify and launch**:
   ```bash
   ./xrobot3 diagnose
   ./xrobot3 run --interface lo --physics raisim
   ```

---

## Project Structure

```text
Xrobot3/
├── xrobot3                     # Main CLI wrapper (auto-bootstraps .venv & env.sh)
├── cli.py                      # Unified CLI (run, check, diagnose, sync, deploy, upload, body, remote, interfaces)
├── network.py                  # IPv4 NIC discovery & FastRTPS DDS XML profile generator
├── env.sh / install.sh         # Auto-detecting ROS 2 Humble/Jazzy environment installer
├── scripts/
│   ├── setup_mujoco4_dev.sh    # MuJoCo 4.0 (.venv-mj4) source/wheel builder
│   ├── check_onboard_terminal.py
│   ├── check_sim_suspension.py
│   └── check_raisim_backend.py # Offline & decoupling tests for RaiSim + diagnose
├── sdk/
│   ├── rl_control_node*.py     # 50 Hz SDK control nodes (hardware & simulation)
│   ├── FSM/                    # 5-state FSM (STOP / ZERO / FIX / SDK-A / WALK)
│   ├── common/                 # Torque limiter, robot interface, terminal & WebSocket console
│   ├── config/                 # Hardware & simulation YAML configs (50 Hz, STAND default)
│   ├── local_setup/            # Contract verifier (check_sdk_a_hardware.py) & diagnostics (diagnose.py)
│   └── policy/
│       ├── sdk_a_to/           # SDK-A ONNX, deploy.json, JitterMonitor, RenderBridge & RaiSim bridge/worker/URDF
│       ├── dex_amp/            # DEXAMP velocity locomotion policy
│       └── fix/                # Fixed standing posture targets & PD gains
├── xmujoco/                    # MuJoCo simulator view & async DDS bridge
├── assets/                     # Tiangong 29-DoF MJCF scenes & STL/OBJ meshes
└── docs/                       # Engineering & handoff notes
```

---

## Verification & Safety Notes

Recommended pre-release verification suite:

```bash
# 1. Contract verification & full environment diagnostics
./xrobot3 check
./xrobot3 diagnose

# 2. Offline FSM (25 transitions), suspension hotkeys, and RaiSim decoupling checks
source env.sh && python scripts/check_onboard_terminal.py
.venv-mj4/bin/python scripts/check_sim_suspension.py
.venv-mj4/bin/python scripts/check_raisim_backend.py

# 3. SDK updater & headless simulation smoke tests
./xrobot3 sync --dry-run
./xrobot3 run --interface lo --physics mujoco --headless --seconds 5
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7 --headless --seconds 4
```

- Per-run logs, `live_status.json`, trajectories (`trace.npz`), and `"jitter"` summaries are saved under `output/`.
- **Simulation passing does not guarantee hardware safety.** Before running on physical hardware, always verify joint feedback signs/offsets, DDS domain (`0` vs `174`), network interface, emergency stop readiness, and effort limits in [deploy.json](sdk/policy/sdk_a_to/deploy.json).
- Third-party upstream repositories and pinned commits are recorded in [UPSTREAM.json](UPSTREAM.json).
