# Xrobot3 (Xmimic SDK3)

[English Documentation](README.md)

**面向天工 3（Tiangong 3，29 自由度）的统一真机控制、Sim-to-Real 仿真与硬件安全运行时。**

同一套 SDK 控制器、五态状态机（FSM）、PD 力矩限幅器与 ONNX 策略链路，无缝覆盖 **本地 1 kHz 物理仿真**（默认 MuJoCo / 可选 RaiSim）、**有线 Offboard 直连控制** 与 **机载真机部署**（基于 ROS 2 / FastRTPS DDS）。

---

## 核心特性

- **可插拔 1 kHz 物理仿真后端（`--physics mujoco` | `--physics raisim`）**：
  - **默认 MuJoCo 后端（`--physics mujoco`）**：默认以 `1 kHz` 运行 MuJoCo `4.0`（`--mujoco-version 4.0`，基于 `.venv-mj4`），支持一键切回 `--mujoco-version 3.3.7`（`.venv`）；旧参数 `--physics 4.0` / `3.3.7` 保持向下兼容。
  - **可选 RaiSim + Rayrai 后端（`--physics raisim`）**：采用完全解耦设计，支持 **Rayrai** 3D 渲染器（`--raisim-renderer rayrai`）与 `RaisimServer`（`--raisim-renderer tcp`）；未安装 RaiSim 时不影响默认 MuJoCo 与真机任何功能。
- **实时“致命伤机级”高频抖动与冲击监测（`JitterMonitor v3`）**：
  - **50 Hz 零阶保持（ZOH）免疫**：准确区分正常 `1–4 Hz` 步态与 20 ms 策略换帧阶梯，避免误报。
  - **全频段硬件损伤判据**：覆盖 `10–25 Hz` 持续 $I^2t$ 发热/减速器交变疲劳、`30–250 Hz` 窗内 PD/触地自激、ISO 20816 风格 $\ge 10\text{ Hz}$ 速度振动，以及按关节真实 `kp / effort_limit` 折算的指令与力矩冲击。
  - **3D 视觉实时告警**：在 MuJoCo 3D 窗口直接高亮异常关节（`>= 55%` 琥珀色预警，`>= 75%` 红色脉冲致命告警），配合固定量程 `0–105%` 实时示波器与顶部 HUD 横幅。
- **统一五态状态机（`50 Hz` 控制与策略推理 / `1 kHz` 底层 PD）**：
  - 启动默认进入安全 `STOP` 保持态；支持 `STOP`、`ZERO`、`FIX`（2 秒平滑插值站立保持）、`SDK-A`（默认安全启动在 `STAND` `50..240` 站立切片，支持 `FILTER` 滤波与 `REF` 切片切换）及 `WALK`（`DEXAMP` 速度行走，`VEL vx vy [wz]`）。
- **仿真与真机严格对齐的 PD 力矩限幅器**：在 `sdk/common/torque_limiter.py` 中按 `deploy.json` 严格约束 29 个串联关节、并联踝关节电机对（`sptlib`）及 `float32` 报文传输精度。
- **一键远程部署、同步与系统诊断**：内置带进度条的 SSH/rsync 部署（`deploy`、`upload`、`remote`）、远程启停机载 `body_control`（`body`）、SDK 在线更新（`sync`）、契约校验（`check`）与全链路环境诊断（`diagnose`）。
- **双版本 ROS 2 自动适配**：自动识别 ROS 2 **Humble**（Ubuntu 22.04，Python 3.10）与 **Jazzy**（Ubuntu 24.04，Python 3.12）。

---

## 系统架构

```text
          终端交互 CLI / WebSocket 控制台（端口 12888）
                               │
                               ▼
     SDK 控制器与五态 FSM (50 Hz) + ONNX 策略推理 (50 Hz)
                        + PD 力矩限幅器
                               │  ROS 2 / FastRTPS DDS
                               │ （29 关节 CmdMotorCtrl & MotorStatusMsg）
                 ┌─────────────┴─────────────┐
                 ▼                           ▼
          1 kHz 本地物理仿真              天工 3 真机
   • MuJoCo 4.0 / 3.3.7（默认，       • 机载 Onboard / 有线 Offboard
     含 3D HUD 抖动与冲击实时告警）      body_control 驱动（1 kHz PD）
   • RaiSim + Rayrai（可选后端）
```

---

## 快速开始

### 1. 环境安装（Ubuntu 22.04 Humble / 24.04 Jazzy，`x86_64`）

请先安装对应系统版本的 ROS 2，然后依次执行：

```bash
git clone https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
cd Xrobot3
bash install.sh
bash scripts/setup_mujoco4_dev.sh   # 编译安装默认 MuJoCo 4.0 (.venv-mj4)；仅用 3.3.7 可跳过
./xrobot3 check
./xrobot3 diagnose
```

### 2. 运行本地仿真

```bash
# 交互式 3D 仿真（默认：--physics mujoco --mujoco-version 4.0）
./xrobot3 run --interface lo

# 无头模式快速冒烟测试（5 秒）
./xrobot3 run --interface lo --headless --seconds 5

# 如未安装 .venv-mj4，可切换回 MuJoCo 3.3.7 (.venv)
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7

# 可选：RaiSim 物理引擎 + Rayrai 3D 渲染器（安装方式见下文）
./xrobot3 run --interface lo --physics raisim
./xrobot3 run --interface lo --physics raisim --raisim-renderer tcp
./xrobot3 run --interface lo --physics raisim --headless --seconds 5
```

### 3. 连接与部署真机

```bash
# 查看本机可用于 DDS 通信的 IPv4 网卡
./xrobot3 interfaces

# 方式 A：有线 Offboard 直连真机（将 ROBOT_INTERFACE 替换为实际网卡名）
./xrobot3 run --interface ROBOT_INTERFACE

# 方式 B：一键部署到机载电脑、启动 body_control 驱动并远程运行
./xrobot3 deploy --host USER@ROBOT
./xrobot3 body --host USER@ROBOT
./xrobot3 run --host USER@ROBOT

# 测试结束后停止远程机载 body_control
./xrobot3 body --host USER@ROBOT --stop
```

> 仿真默认使用 DDS Domain `174`（`lo`）；真机默认使用 Domain `0`（默认对端 IP `192.168.41.1`，可通过 `--robot-ip` / `--domain` 覆盖）。任意命令均可追加 `--dry-run` 仅预览最终执行指令而不实际运行。

---

## 交互终端指令

仿真与机载/有线真机终端共用同一套不区分大小写的控制指令：

| 指令 | 说明 |
| :--- | :--- |
| `STOP` | 进入 SDK `STOP` 保持控制器（启动默认状态，非零阻尼/保持力矩） |
| `ZERO` | 平滑回到预设零位/准备姿态 |
| `FIX` / `STAND` | 2 秒内平滑插值到固定站立姿态并 PD 保持（不运行 ONNX） |
| `SDK-A` | 启动 SDK-A Tracking / Recovery ONNX 策略（默认运行在安全站立切片 `STAND` `50..240`） |
| `FILTER [参数]` | 开关或调节 SDK-A 动作 EMA 滤波（`ON` / `OFF` / `LIGHT` / `MED` / `STRONG` / `<alpha>`） |
| `REF [参数]` | 切换 SDK-A 参考动作切片（`STAND` / `DANCE` / `FULL` / `<start> <end>`） |
| `WALK` | 启动 `DEXAMP` 速度行走策略（初始零速） |
| `VEL vx vy [wz]` | 在 `WALK` 状态下实时更新目标前向/侧向/偏航速度（`m/s`、`rad/s`） |
| `HANG ON\|OFF\|UP\|DOWN\|STRONGER\|WEAKER` | *（仅仿真可用）* 虚拟弹性吊绳保护（快捷键说明见[仿真吊绳快捷键](#仿真吊绳快捷键)） |

### 仿真吊绳快捷键

> **所需窗口焦点：** 键盘快捷键仅在**当前 3D 仿真窗口**（MuJoCo 窗口或 RaiSim Rayrai 窗口）获得键盘焦点时生效——请先用鼠标点击 3D 窗口内部。当终端窗口获得焦点时，请直接输入 `HANG ON|OFF|UP|DOWN|STRONGER|WEAKER`。

| 操作 | 3D 窗口快捷键（MuJoCo 与 Rayrai 通用） | 说明 |
| :--- | :--- | :--- |
| 启用吊绳支撑 | `F9` / `9` / `Insert`（释放状态下按 `↑` 也会自动启用） | 开启虚拟弹性吊绳支撑（`ENABLED`） |
| 抬升 / 下降机器人 | `↑` / `↓`（亦支持 `PageUp`/`PageDown`、`7`/`8`、小键盘 `7`/`9` 与 `1`/`3`） | 每次按键调整目标悬吊高度 `±0.05 m` |
| 一键释放吊绳 | `F10` / `0` / `Delete` | 单次按键立即关闭吊绳支撑（`RELEASED (OFF)`） |
| 切换启用 / 释放 | `H` / 小键盘 `0` / 小键盘 `5` | 在 `ENABLED` 与 `RELEASED (OFF)` 之间切换 |
| 增强 / 减弱刚度 | `.` / `KP+`（`KP8`/`KP6`）与 `,` / `KP-`（`KP2`/`KP4`） | 调节弹簧刚度（`N/m`）；低于 `65 N/m` 自动释放（`-`/`=` 留给 MuJoCo 仿真调速） |

- **窗口状态反馈：** MuJoCo 左上角 HUD 实时显示 `Suspension: ENABLED (z=...m, k=...N/m)` 或 `RELEASED (OFF)` 及快捷键提示；Rayrai 在窗口标题栏实时显示吊绳状态、渲染 3D 绿色吊绳连杆，并提供 ImGui 交互滑条（同时支持 `Space` 暂停、`R` 复位姿态）。

---

## CLI 子命令一览

| 命令 | 用途 |
| :--- | :--- |
| `./xrobot3 run` | 启动本地仿真（`--interface lo`）、有线 Offboard（`--interface 网卡`）或 SSH 远程机载控制（`--host USER@ROBOT`） |
| `./xrobot3 check` | 校验 Python 环境、ROS 2 消息包、力矩限幅器以及 29 关节 ONNX / `deploy.json` / URDF 契约 |
| `./xrobot3 diagnose` | 一键诊断 ROS 2、GPU 功耗/温控、DDS 网卡、默认 MuJoCo 与可选 RaiSim 就绪状态（支持 `--json`、`--strict-raisim`） |
| `./xrobot3 interfaces` | 列出本机可用于 FastRTPS DDS 通信的 IPv4 网卡 |
| `./xrobot3 sync` | 带进度条从 `main` 分支自动更新最新 SDK 文件（兼容有无 `.git` 环境；支持 `--reinstall`、`--tarball`、`--dry-run`） |
| `./xrobot3 deploy --host USER@ROBOT` | 通过 `rsync`（带进度条与 ETA）同步工作区并在机载端执行 `install.sh` |
| `./xrobot3 upload --host USER@ROBOT` | 仅快速增量同步代码文件到机载端（不重复执行 `install.sh`） |
| `./xrobot3 body --host USER@ROBOT [--stop]` | 远程启动或停止机载实时驱动 `body_control` |
| `./xrobot3 remote --host USER@ROBOT` | 通过 SSH 在机载端直接拉起 SDK 控制会话 |

常用仿真参数：`--physics {mujoco,raisim}`（默认 `mujoco`）、`--mujoco-version {4.0,3.3.7}`（默认 `4.0`）、`--raisim-renderer {rayrai,tcp}`（默认 `rayrai`）、`--headless`、`--seconds N`、`--domain ID`、`--dry-run`。

---

## 可选：安装与启用 RaiSim + Rayrai（`--physics raisim`）

> **完全解耦设计：** RaiSim 为**纯可选后端**。即使本机未安装 `raisim2Lib` 或未配置授权文件，默认 `./xrobot3 run --interface lo`（`--physics mujoco`）、`./xrobot3 check`、`./xrobot3 diagnose` 以及真机控制均正常可用、零报错；若在未就绪环境下显式指定 `--physics raisim`，CLI 会在启动前即时拦截并输出清晰的安装/授权指引与 `--physics mujoco` 回退命令。

1. **安装编译依赖**（Ubuntu 22.04 / 24.04）：
   ```bash
   sudo apt-get update && sudo apt-get install -y cmake g++ libeigen3-dev libsdl2-dev python3-venv
   ```
2. **克隆 `raisim2Lib` 并编译 `raisimPy` 与 `rayrai`**（在独立 `.venv` 中构建，确保与 ROS 2 Humble 的 Python 3.10 环境隔离）：
   ```bash
   git clone https://github.com/BenHuHuan/raisim2Lib.git ../raisim2Lib
   cd ../raisim2Lib
   python3 -m venv .venv && source .venv/bin/activate
   pip install numpy
   source ./raisim_env.sh
   cmake -S . -B build-py -DCMAKE_BUILD_TYPE=Release -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON
   cmake --build build-py --parallel
   ```
   > 运行时自动按 `$RAISIM2LIB_ROOT`、同级目录 `../raisim2Lib`、`~/raisim2Lib` 查找（同时兼容 `build-py/raisimPy` 与 `build/raisimPy` 构建目录，亦兼容上游 `raisimTech/raisim2Lib`）。若安装在其他路径，请设置 `export RAISIM2LIB_ROOT=/path/to/raisim2Lib`。
3. **配置授权文件 `activation.raisim`**：
   - 前往 [RaiSim 官方授权申请表单](https://forms.gle/4bcS2GByKYdeKxmn7) 免费申请 Academic 或 Trial 授权。
   - 将收到的文件保存为 `~/.raisim/activation.raisim`（或设置环境变量 `export RAISIM_ACTIVATION_KEY=/path/to/activation.raisim`）。
4. **诊断验证并启动**：
   ```bash
   ./xrobot3 diagnose
   ./xrobot3 run --interface lo --physics raisim
   ```

---

## 目录结构

```text
Xrobot3/
├── xrobot3                     # 主入口脚本（自动初始化/加载 .venv 与 env.sh）
├── cli.py                      # 统一 CLI（run, check, diagnose, sync, deploy, upload, body, remote, interfaces）
├── network.py                  # IPv4 网卡识别与 FastRTPS DDS 配置文件生成
├── env.sh / install.sh         # 自动适配 ROS 2 Humble/Jazzy 的安装与环境脚本
├── scripts/
│   ├── setup_mujoco4_dev.sh    # MuJoCo 4.0 (.venv-mj4) 编译安装脚本
│   ├── check_onboard_terminal.py
│   ├── check_sim_suspension.py
│   └── check_raisim_backend.py # RaiSim 后端、IPC 与解耦诊断离线测试
├── sdk/
│   ├── rl_control_node*.py     # 50 Hz SDK 控制节点（真机与仿真共用）
│   ├── FSM/                    # 五态状态机（STOP / ZERO / FIX / SDK-A / WALK）
│   ├── common/                 # 力矩限幅器、机器人通信接口、终端与 WebSocket 控制台
│   ├── config/                 # 真机与仿真 YAML 配置（50 Hz，默认 STAND 安全切片）
│   ├── local_setup/            # 硬件契约校验（check_sdk_a_hardware.py）与系统诊断（diagnose.py）
│   └── policy/
│       ├── sdk_a_to/           # SDK-A 策略、deploy.json、JitterMonitor、RenderBridge 与 RaiSim 桥接/渲染器
│       ├── dex_amp/            # DEXAMP 速度行走策略
│       └── fix/                # FIX 站立姿态目标与 PD 增益配置
├── xmujoco/                    # MuJoCo 仿真窗口与异步 DDS 桥接
├── assets/                     # 天工 29 自由度 MJCF 场景与 STL/OBJ 网格资产
└── docs/                       # 工程交接与技术说明文档
```

---

## 验证与安全提示

推荐的发布前验证流程：

```bash
# 1. 部署契约校验与全系统环境诊断
./xrobot3 check
./xrobot3 diagnose

# 2. 离线 25 态状态机切换、吊绳快捷键与 RaiSim 解耦回归测试
source env.sh && python scripts/check_onboard_terminal.py
.venv-mj4/bin/python scripts/check_sim_suspension.py
.venv-mj4/bin/python scripts/check_raisim_backend.py

# 3. SDK 更新预检与 MuJoCo 4.0 / 3.3.7 无头冒烟测试
./xrobot3 sync --dry-run
./xrobot3 run --interface lo --physics mujoco --headless --seconds 5
./xrobot3 run --interface lo --physics mujoco --mujoco-version 3.3.7 --headless --seconds 4
```

- 每次运行的终端日志、`live_status.json`、轨迹数据（`trace.npz`）及抖动统计（`"jitter"` 字段）均保存在 `output/` 目录下。
- **仿真通过不等于真机安全**：上真机前务必核对关节反馈符号与零偏、DDS Domain（真机 `0` vs 仿真 `174`）、网卡绑定、急停保护就绪状态，以及 [deploy.json](sdk/policy/sdk_a_to/deploy.json) 中的力矩限幅参数。
- 第三方上游仓库与锁定 Commit 记录见 [UPSTREAM.json](UPSTREAM.json)。
