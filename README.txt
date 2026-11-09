Xrobot3 — X robot 3 unified control and simulation
=====================================================

One SDK controller, FSM and ONNX policy path for local simulation,
onboard control and wired offboard control. The simulator supplies joint
and IMU feedback through ROS 2 / DDS; the SDK sends motor targets back.

Installation (Ubuntu 22.04, x86_64, Python 3.10, ROS 2 Humble)
---------------------------------------------------------
Install ROS 2 Humble and uv first, then:
  git clone https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
  cd Xrobot3
  bash install.sh
  ./xrobot3 check
The SDK and MuJoCo use the same .venv. Bundled bodyctrl messages are
extracted into vendor/; no system packages are overwritten.

Run
---
Interactive local simulation:
  ./xrobot3 run --mode offboard --interface lo
Wired real robot (default peer 192.168.41.1, DDS domain 0):
  ./xrobot3 run --target real --mode offboard --interface enp58s0
Override peer if needed: --robot-ip ADDRESS
On the robot:
  ./xrobot3 run --target real --mode onboard
Upload and launch over SSH:
  ./xrobot3 upload --host USER@ROBOT
  ./xrobot3 remote --host USER@ROBOT
Run install.sh on the robot after the first upload. The bundled binary
packages target x86_64 Ubuntu 22.04; other platforms need matching packages.

Simulation defaults to DDS domain 174 and loopback; real defaults to 0.
Both communicating endpoints must use the same domain. A domain is a DDS
communication group, not a robot ID. --dry-run prints commands without
launching. --headless --seconds 5 runs a short simulation.

Terminal commands (case insensitive)
------------------------------------
  STOP              Hold using the SDK STOP controller (not zero torque).
  ZERO              Move to the configured ready posture.
  SDK-A             Run the SDK-A Tracking / Recovery ONNX policy.
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
frequency, mean FSM computation time and viewer FPS. FSM time includes
state transitions and policy computation; it is not isolated ONNX latency.
Statistics refresh over approximately 0.5 s. Missing SDK telemetry is shown
as waiting/zero. Rendering runs separately and does not lock the solver.
Physics uses 1 ms steps; SDK-A and DEXAMP run at 50 Hz. The real motor servo
rate is determined by the robot firmware, not the simulator setting.

Model and deployment contract
-----------------------------
sdk/policy/sdk_a_to contains the ONNX, reference and deployment contract.
SDK-A controls 29 body joints; head joints retain the configured behavior.
Model hashes, PD gains, scales and joint ordering are checked by ./xrobot3 check.
This integration does not include SafeFall, B_PLUS or GroundBridge.
Simulation testing is not hardware validation. Check the robot's feedback
frames, network and actuator configuration before running on hardware.

Source provenance
-----------------
See UPSTREAM.json for upstream URLs and pinned commits. SDK and simulator
sources are vendored with local integration changes. Their original source
and dependency licensing terms continue to apply; no replacement license
is asserted for third-party code or model assets.
