Xrobot3 — X robot 3 unified control and simulation
=====================================================

One SDK controller, FSM and ONNX policy path for local simulation,
onboard control and wired offboard control. The simulator supplies joint
and IMU feedback through ROS 2 / DDS; the SDK sends motor targets back.

Recent updates (2026-11)
------------------------
- Dual ROS 2 / Ubuntu support: install.sh and env.sh now auto-detect ROS 2
  Jazzy (Ubuntu 24.04, Python 3.12), ROS 2 Humble (Ubuntu 22.04, Python 3.10),
  and onboard vendor xos setups, unpacking matching bodyctrl_msgs and
  sptlib_python packages from sdk/lib/22 or sdk/lib/24.
- One-command remote deploy & body driver CLI: added `./xrobot3 deploy`
  (rsync with live progress/ETA + remote install.sh), `./xrobot3 body
  --host USER@ROBOT [--stop]` (start/stop onboard real-time body_control),
  and `./xrobot3 interfaces` with per-adapter FastRTPS DDS profile generation.
- 100 Hz SDK control loop & initial pose latch: upgraded SDK control cadence
  to 100 Hz (dt=0.01 s) while clocking ONNX policy inference at 50 Hz, and
  latched initial sensor feedback before publishing the first motor command.
- Simulation-aligned PD torque limiter: added sdk/common/torque_limiter.py
  enforcing deploy.json effort bounds across 29 serial joints, parallel ankle
  motor pairs (direction-preserving sptlib wrench scaling), and float32 wire
  commands on both simulation and hardware.
- Unified 5-state FSM & live SDK-A controls: added FIX (2 s interpolated
  standing hold) alongside STOP, ZERO, SDK-A and WALK (Unitree-style VEL vx
  vy [wz]), plus live terminal/web controls for EMA action smoothing
  (FILTER ON/OFF/LIGHT/MED/STRONG/<alpha>) and reference motion slicing
  (REF STAND/DANCE/FULL/<start> <end>).
- Protection stack runtime hook: added optional 50 Hz closed-loop
  ProtectionStackRuntime (sdk/policy/sdk_a_to/protection_runtime.py) for
  coordinating SDK-A with B_PLUS, SafeFall and AutonomousGroundBridge bundles.
- Upgraded MuJoCo interactive viewer & elastic suspension: integrated native
  MuJoCo Simulate controls (pause, reset, history scrub, perturbation, RTF)
  with virtual torso suspension (HANG ON/OFF/UP/DOWN/STRONGER/WEAKER and
  keyboard shortcuts) and live HUD telemetry.

Installation (Ubuntu 22.04 / Humble or Ubuntu 24.04 / Jazzy, x86_64)
--------------------------------------------------------------------
Install ROS 2 (Humble or Jazzy) first, then:
  git clone https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
  cd Xrobot3
  bash install.sh
  ./xrobot3 check
install.sh uses uv (or falls back to python3 -m venv) so the SDK and MuJoCo
share one .venv (Python 3.10 on Humble, Python 3.12 on Jazzy/xos). Bundled
bodyctrl messages are extracted into vendor/; no system packages are
overwritten.

Run
---
Interactive local simulation:
  ./xrobot3 run --interface lo
Wired real robot (default peer 192.168.41.1, DDS domain 0):
  ./xrobot3 run --interface ROBOT_INTERFACE
Override peer if needed: --robot-ip ADDRESS
On the robot:
  ./xrobot3 run --target real --mode onboard
Deploy, start body driver, and launch over SSH:
  ./xrobot3 deploy --host USER@ROBOT
  ./xrobot3 body --host USER@ROBOT
  ./xrobot3 run --host USER@ROBOT
Stop onboard body_control over SSH when finished:
  ./xrobot3 body --host USER@ROBOT --stop
The bundled binary packages target x86_64 Ubuntu 22.04 and 24.04; other
platforms need matching packages.

Simulation defaults to DDS domain 174 and loopback; real defaults to 0.
Both communicating endpoints must use the same domain. A domain is a DDS
communication group, not a robot ID. --dry-run prints commands without
launching. --headless --seconds 5 runs a short simulation.

Terminal commands (case insensitive)
------------------------------------
  STOP              Hold using the SDK STOP controller (not zero torque).
  ZERO              Move to the configured ready posture.
  FIX               Interpolate to fixed standing posture in 2 s and hold.
  SDK-A             Run the SDK-A Tracking / Recovery ONNX policy.
  FILTER [arg]      Toggle or set SDK-A EMA action filter (ON/OFF/LIGHT/MED/STRONG/alpha).
  REF [arg]         Select SDK-A reference slice (STAND/DANCE/FULL or <start> <end>).
  WALK              Run the bundled DEXAMP walking policy, zero velocity.
  VEL 0.3 0         Set forward/lateral velocity; yaw defaults to zero.
  VEL 0 0 0.3       Set yaw velocity.
  VEL 0 0 0         Zero walking commands.
Commands are acknowledged after the FSM and command-send step, not merely
when queued. Rejections are reported. Detailed logs stay in
output/terminal_logs/ and do not overwrite terminal input.

Simulation performance display
------------------------------
The viewer shows simulation time, real-time factor, measured physics steps
per wall-clock second, configured physics frequency, measured SDK control
frequency, mean FSM computation time, viewer FPS, active FSM state, action
filter status and suspension status. FSM time includes state transitions and
policy computation; it is not isolated ONNX latency. Statistics refresh over
approximately 0.5 s. Rendering runs separately and does not lock the solver.
Physics uses 1 ms steps; the SDK control loop runs at 100 Hz while SDK-A and
DEXAMP infer at 50 Hz. The real motor servo rate is determined by the robot
firmware, not the simulator setting.

Model, torque limits and deployment contract
--------------------------------------------
sdk/policy/sdk_a_to contains the ONNX, reference and deployment contract.
SDK-A controls 29 body joints; head joints retain the configured behavior.
Model hashes, PD gains, scales, torque limits and joint ordering are checked
by ./xrobot3 check. Simulation-aligned PD torque limiting (deploy.json effort)
is enabled by default across serial joints, parallel ankle pairs and float32
wire serialization. Optional 50 Hz ProtectionStackRuntime hooks (SafeFall,
B_PLUS and GroundBridge) are provided in protection_runtime.py when external
protection bundles are configured; those extra ONNX bundles are not bundled by
default. Simulation testing is not hardware validation. Check the robot's
feedback frames, network and actuator configuration before running on hardware.

Source provenance
-----------------
See UPSTREAM.json for upstream URLs and pinned commits. SDK and simulator
sources are vendored with local integration changes. Their original source
and dependency licensing terms continue to apply; no replacement license
is asserted for third-party code or model assets.

Simulation suspension
---------------------
  HANG ON            Enable elastic torso support (minimum target height 1.2 m).
  HANG UP            Raise the suspension target by 0.1 m.
  HANG DOWN          Lower the suspension target by 0.1 m.
  HANG OFF           Remove support and return to free physics.
Disabled by default. Suspension status remains available in output/live_status.json for each run. This is a virtual
spring-damper with gravity support, not a rigid root lock or a robot FSM.
It does not constrain joint motion or guarantee upright orientation.
Hardware endpoints reject HANG. Use HANG OFF for walking/recovery evaluation.

Velocity locomotion model
-------------------------
WALK loads policy/dex_amp/model/dex23_amp_isaaclab_terrain_base_48700.onnx
from https://github.com/Dknt0/xmigcs_dex3 . It uses vx, vy and yaw-rate,
23 actions, 29 motor targets, 8 x 78 observations, and 50 Hz inference.
The remaining six joints use configured hold targets. The supplied model
is a community deployment model, not a certified official hardware policy.
Official sources inspected: Open-X-Humanoid/xhumanoid_sdk,
Open-X-Humanoid/Deploy_Tienkung, Open-X-Humanoid/TienKung-Lab.
No drop-in Tiangong 3 ONNX replacement was found in those inspected trees.

FIX state (simulation and hardware)
-----------------------------------
  FIX               Interpolate to a fixed standing target in 2 seconds,
                    then hold with PD. No ONNX policy is run in this state.
  WALK / SDK-A      Leave FIX and enter the requested policy.
  STOP              Leave FIX immediately for STOP.
FIX follows the Unitree FixStand pattern and uses Tiangong-specific targets
and gains in sdk/policy/fix/config.json. It does not lock the floating base
or supply external support. It is distinct from simulation-only HANG.

Unitree-style state and velocity control
---------------------------------------
FIX / FIXSTAND / STAND enter fixed-standing control (2 s interpolation).
WALK enters velocity locomotion at zero speed. It accepts no arguments.
VEL vx vy [wz] updates speed only while WALK is active; yaw defaults to zero.
Velocity updates do not restart the policy/history or switch FSM states.
Leaving WALK through STOP, ZERO, FIX or SDK-A clears the velocity commands.

Viewer suspension shortcuts (focus the MuJoCo window)
-----------------------------------------------------
F9: HANG ON; F10: HANG OFF; main-row 7/8: raise/lower target 0.1 m.
Arrow Up: stronger spring (x1.25); Arrow Down: weaker spring (x0.8).
Stiffness is limited to 250..4000 N/m, with damping adjusted accordingly.
Terminal equivalents: HANG ON/OFF/UP/DOWN/WEAKER/STRONGER.
The run live_status.json records assistance, target height and stiffness. Spring stiffness
changes suspension compliance, not motor PD gains or policy action scaling.

Viewer layout: native MuJoCo Simulate panels (pause/run, reset, history
scrubbing, real-time speed and perturbation forces) synchronize bidirectionally
with the simulation process alongside the HUD performance overlay and hotkeys.

Onboard terminal parity
-----------------------
./xrobot3 run --target real --mode onboard uses the same stdin commands,
FSM and ONNX policies as simulation. STOP/ZERO/FIX/SDK-A/WALK, VEL, FILTER
and REF work without a graphical display. Only HANG is simulation-only and is
hidden from the hardware prompt. Detailed runtime logs remain separate from stdin.
Offline verification (no ROS node or actuator publishing):
  source env.sh
  python scripts/check_onboard_terminal.py
This checks all 25 transitions among the five states using the hardware
configuration, velocity updates/clearing, FILTER/REF commands and rejection of
HANG. It does not replace verification on the actual onboard computer and
robot interfaces.

Selecting the robot connection
-----------------------------
List this computer's available adapters:
  ./xrobot3 interfaces
Local simulation (same native SDK controller; MuJoCo is the feedback peer):
  ./xrobot3 run --interface lo
Onboard terminal:
  ./xrobot3 run --target real --mode onboard
Wired offboard (replace the adapter with the name listed on your computer):
  ./xrobot3 run --interface ROBOT_INTERFACE
Wi-Fi access to the robot means SSH into its onboard computer, then run the
SDK there (onboard control); it is not a separate wireless offboard mode:
  ./xrobot3 run --host USER@ROBOT
Or, after SSH login, select the robot-facing adapter on that computer:
  ./xrobot3 run --interface ROBOT_INTERFACE
ROBOT_INTERFACE is a placeholder: use the actual adapter name from
./xrobot3 interfaces. Only lo selects simulation; every other interface
selects the real robot configuration. --target and --mode are unnecessary.
The default real peer is 192.168.41.1 if --robot-ip is omitted.
Both endpoints must share the DDS domain (real default 0).
Use --dry-run to inspect any launch without starting a controller.

Remote run options: --port SSH_PORT, --path PROJECT_PATH, --domain DOMAIN.
With --host, --interface refers to the onboard robot-facing adapter, not the
laptop Wi-Fi adapter. SSH carries the interactive terminal; SDK inference
and ROS2/DDS motor communication run on the onboard computer.

One-command onboard installation
--------------------------------
  ./xrobot3 deploy --host USER@ROBOT
  ./xrobot3 body --host USER@ROBOT
  ./xrobot3 run --host USER@ROBOT

Deploy uploads the project (with live transfer progress and ETA) and runs
install.sh over SSH. It excludes local virtual environments, logs and outputs;
it does not remove remote files or start motor control. --port and --path are
supported. Use deploy --dry-run to inspect commands without connecting.

install.sh finds or installs uv (or falls back to python3 -m venv), creates a
matching .venv (Python 3.10 on Humble / Ubuntu 22.04; Python 3.12 on Jazzy /
Ubuntu 24.04 / vendor xos), installs pinned requirements and bundled vendor
packages, then checks runtime imports and ONNX contracts without publishing
motor commands. Re-running it reuses the project environment. A local first
launch installs if .venv is missing. No conda activation or manual environment
activation is needed. Internet is needed on first installation; this script
does not replace the robot's system drivers or install body_control.
