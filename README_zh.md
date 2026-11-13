# Xrobot3 (Xmimic SDK3)

[English Documentation](README.md)

**面向天工 3（Tiangong 3，29 自由度）的统一真机控制、Sim-to-Real 仿真与硬件安全运行时。**

一套 SDK 控制器、五态状态机（FSM）、PD 力矩限幅器与 ONNX 策略链路，同时覆盖 **本地 MuJoCo 仿真**、**有线 Offboard 控制** 与 **机载真机部署**（基于 ROS 2 / FastRTPS DDS）。

---

## 核心特性

- **默认启用 MuJoCo 4.0 仿真（`--physics 4.0`）**：物理步长 `1 kHz`，默认使用 `.venv-mj4`（MuJoCo 4.0），支持通过 `--physics 3.3.7` 一键回退至 `.venv`（MuJoCo 3.3.7）。
- **实时“致命伤机级”高频抖动与冲击监测（JitterMonitor v3）**：
  - **50 Hz 零阶保持（ZOH）免疫**：不会把正常 `1–4 Hz` 步态或 20 ms 策略换帧阶梯误报为毛刺。
  - **全频段硬件损伤判据**：覆盖 `10–25 Hz` 持续 $I^2t$ 发热/减速器交变疲劳、`30–250 Hz` 盒内局部 PD/触地自激、ISO 20816 风格 `>= 10 Hz` 速度振动，以及按各关节真实 `kp / effort_limit` 折算的指令与力矩冲击。
  - **3D 视觉告警**：在 MuJoCo 渲染窗口直接高亮故障关节（`>= 55%` 琥珀预警，`>= 75%` 红色脉冲致命告警），配合固定 `0–105%` 实时示波器与顶部 HUD 横幅。
- **统一五态状态机（`100 Hz` 控制循环 / `50 Hz` 策略推理）**：
  - 支持 `STOP`、`ZERO`、`FIX`（2 秒平滑站立保持）、`SDK-A`（跟踪/恢复策略，支持 `FILTER` 动作滤波与 `REF` 切片）及 `WALK`（`DEXAMP` 速度行走，`VEL vx vy [wz]`）。
- **仿真真机对齐力矩限幅器**：在 `sdk/common/torque_limiter.py` 中按 `deploy.json` 严格限制 29 个关节及并联踝关节电机对（`sptlib`）的力矩与 `float32` 传输精度。
- **一键远程部署与驱动管理**：内置 SSH/rsync 带进度条部署（`deploy`）、远程启停机载 `body_control`（`body`）、网卡发现（`interfaces`）与环境/策略契约校验（`check`）。
- **双版本 ROS 2 兼容**：自动识别 ROS 2 **Humble**（Ubuntu 22.04, Python 3.10）与 **Jazzy**（Ubuntu 24.04, Python 3.12）。

---

## 系统架构

```text
  终端控制 / WebSocket 控制台（端口 12888）
                 │
                 ▼
  SDK 控制器 (100 Hz) + ONNX 策略 (50 Hz) + 力矩限幅器
                 │  ROS 2 / FastRTPS DDS（29 关节 CmdMotorCtrl / MotorStatusMsg）
        ┌────────┴────────┐
        ▼                 ▼
  MuJoCo 4.0 仿真    天工 3 真机
  (1 kHz + 3D 抖动   (机载 Onboard / 有线 Offboard
   / 冲击实时告警)    body_control 驱动)
```

---

## 快速开始

### 1. 环境安装（Ubuntu 22.04 Humble / 24.04 Jazzy，`x86_64`）

请先安装对应版本的 ROS 2，然后执行：

```bash
git clone -b dev https://github.com/BenHuHuan/Xmimic_sdk3.git Xrobot3
cd Xrobot3
bash install.sh
./xrobot3 check
```

配置 MuJoCo 4.0 仿真环境（`.venv-mj4`）：

```bash
bash scripts/setup_mujoco4_dev.sh
```

### 2. 运行真机仿真

```bash
# 交互式 3D 仿真（默认 MuJoCo 4.0）
./xrobot3 run --interface lo

# 如需切换回 MuJoCo 3.3.7
./xrobot3 run --interface lo --physics 3.3.7

# 无头模式快速冒烟测试（5 秒）
./xrobot3 run --interface lo --headless --seconds 5
```

### 3. 连接与部署真机

```bash
# 查看本机可用 IPv4 网卡
./xrobot3 interfaces

# 有线 Offboard 直连真机（替换 ROBOT_INTERFACE 为实际网卡名）
./xrobot3 run --interface ROBOT_INTERFACE

# 一键部署到机载电脑、启动 body_control 并远程运行
./xrobot3 deploy --host USER@ROBOT
./xrobot3 body --host USER@ROBOT
./xrobot3 run --host USER@ROBOT

# 结束测试后停止远程 body_control
./xrobot3 body --host USER@ROBOT --stop
```

> 仿真默认 DDS Domain 为 `174`（`lo`）；真机默认 Domain 为 `0`（默认对端 IP `192.168.41.1`，可用 `--robot-ip` 覆盖）。任意命令均可加 `--dry-run` 仅预览实际执行指令。

---

## 交互终端指令（不区分大小写）

仿真与机载/有线真机终端共用同一套指令：

| 指令 | 说明 |
| :--- | :--- |
| `STOP` | 进入 SDK STOP 保持控制器（非零力矩） |
| `ZERO` | 回到预设初始准备姿态 |
| `FIX` / `STAND` | 2 秒内平滑插值到固定站立姿态并 PD 保持（不运行 ONNX） |
| `SDK-A` | 启动 SDK-A Tracking / Recovery ONNX 策略 |
| `FILTER [参数]` | 开关或设置 SDK-A 动作 EMA 滤波（`ON` / `OFF` / `LIGHT` / `MED` / `STRONG` / `<alpha>`） |
| `REF [参数]` | 选择 SDK-A 参考动作切片（`STAND` / `DANCE` / `FULL` / `<start> <end>`） |
| `WALK` | 启动 `DEXAMP` 速度行走策略（初始零速） |
| `VEL vx vy [wz]` | 在 `WALK` 状态下实时更新前向/侧向/偏航速度 |
| `HANG ON\|OFF\|UP\|DOWN\|STRONGER\|WEAKER` | *（仅仿真可用）* 虚拟弹性吊绳保护（快捷键：`F9`/`F10`、`7`/`8`、方向键 `Up`/`Down`） |

---

## CLI 子命令一览

| 命令 | 用途 |
| :--- | :--- |
| `./xrobot3 run` | 启动本地仿真、有线 Offboard 或 `--host` 远程机载控制 |
| `./xrobot3 deploy --host USER@ROBOT` | 通过 `rsync`（带进度条/ETA）同步代码并远程执行 `install.sh` |
| `./xrobot3 body --host USER@ROBOT [--stop]` | 远程启动或停止机载 `body_control` 驱动 |
| `./xrobot3 interfaces` | 列出本机可用于 DDS 通信的 IPv4 网卡 |
| `./xrobot3 check` | 校验 Python 环境、ROS 2 消息包及 ONNX / `deploy.json` 契约 |

---

## 目录结构

```text
Xrobot3/
├── xrobot3                     # 主启动脚本
├── cli.py                      # 统一 CLI 入口（run, deploy, body, check, interfaces）
├── network.py                  # 网卡识别与 FastRTPS DDS 配置生成
├── env.sh / install.sh         # 自动适配 ROS 2 Humble/Jazzy 的安装与环境脚本
├── scripts/
│   ├── setup_mujoco4_dev.sh    # MuJoCo 4.0 (.venv-mj4) 一键安装脚本
│   └── check_onboard_terminal.py
├── sdk/
│   ├── rl_control_node*.py     # 100 Hz SDK 控制节点（真机与仿真）
│   ├── FSM/                    # 五态状态机控制
│   ├── common/                 # 力矩限幅器、机器人接口、终端与 WebSocket 控制台
│   ├── config/                 # 真机与仿真 YAML 配置
│   └── policy/
│       ├── sdk_a_to/           # SDK-A 策略、deploy.json 契约、JitterMonitor 与 RenderBridge
│       ├── dex_amp/            # DEXAMP 速度行走策略
│       └── fix/                # FIX 站立目标与 PD 增益配置
├── xmujoco/                    # MuJoCo 仿真视图与异步通信桥
├── assets/                     # 机器人场景与可视化资产
└── docs/                       # 工程交接与技术文档
```

---

## 验证与安全提示

- 离线状态机与终端指令切换自检：
  ```bash
  source env.sh && python scripts/check_onboard_terminal.py
  ```
- 每次运行的详细日志与抖动统计（`"jitter"` 字段）保存在 `output/` 目录下。
- **仿真通过不等于真机安全**：上真机前务必核对关节反馈坐标系、DDS Domain、网卡配置以及 [deploy.json](sdk/policy/sdk_a_to/deploy.json) 中的力矩限幅参数。
- 第三方上游仓库与锁定 Commit 见 [UPSTREAM.json](UPSTREAM.json)。
