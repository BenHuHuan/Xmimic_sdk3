# Gemini 近期工作交接文档（给 Codex，2026-11-13）

> **生成时间**：2026-11-13 05:13 (+01:00)  
> **覆盖周期**：2026-11-10 ~ 2026-11-13（基于 `Xrobot3` 当前仓库代码、Git 历史、对话记录及本机磁盘实时校验生成；**不保存任何账号密码**）  
> **主要工作区**：`/home/huan/Xrobot3`（软链指向 `/media/huan/本地磁盘/Xrobot3`）与关联训练仓 `/media/huan/本地磁盘/stable_mimic`

---

## 一、 当前仓库、设备与环境基线

### 1. 代码仓与磁盘位置

| 项目 / 资产 | 实际路径 | 当前状态说明 |
|---|---|---|
| **Xrobot3（仿真 + 真机统一 SDK）** | `/home/huan/Xrobot3` → `/media/huan/本地磁盘/Xrobot3` | `HEAD` 为 `6209b61b`（2026-11-11）；工作区有 **10 个已修改文件 + 2 个未跟踪文件**（见第二节） |
| **StableMimic（上游训练与导出脚本）** | `/media/huan/本地磁盘/stable_mimic` | **本机无 `.venv`**，导出 ONNX 需在远端服务器执行或另建环境 |
| **本地仿真 Python 环境（stable）** | `/home/huan/Xrobot3/.venv` | Python 3.10 + ROS 2 Humble + MuJoCo 3.3.7 |
| **本地仿真 Python 环境（dev）** | `/home/huan/Xrobot3/.venv-mj4` | Python 3.10 + GCC 源码编译版 MuJoCo 4.0.0（由 `scripts/setup_mujoco4_dev.sh` 构建） |

### 2. 机器人板载机与远端训练机

| 节点 | 连接标识 | 系统 / 运行环境 | 关键目录 / 备注 |
|---|---|---|---|
| **天工3 机器人板载机** | `ubuntu@192.168.41.1`（有线）<br>`ubuntu@192.168.151.220`（Wi-Fi） | Ubuntu **24.04.3 LTS** (x86_64), **Python 3.12.3**, `/opt/ros/jazzy` + `/home/ubuntu/xos` | 机器人局域网无外网；工作目录 `~/Xrobot3` |
| **双 RTX 5090 训练机** | `eirira2@10.4.156.250` | 单卡 32 GiB（`32,607 MiB`） | `/data/eirira2/stable_mimic`；2026-11-12 已在此完成 `TO17 35k` 续训 |
| **双 RTX PRO 6000 训练机** | `frp-huan`（`Huan@frp.dknt123.cn:36666`） | 双卡 96 GiB | `/data/Huan`；2026-11-12 下午曾因远端机器/frpc 失联转至 5090 续训 |

---

## 二、 代码变更全貌（切勿误删或回滚未提交改动）

### 1. Commit `6209b61b`（2026-11-11）已合入的核心改造
1. **双 ROS 2 / 双 Python ABI 自动部署**（`install.sh`, `env.sh`, `cli.py`）：
   - 自动识别 ROS 2 Jazzy（Ubuntu 24.04 / Py 3.12，解包 `sdk/lib/24/`）、ROS 2 Humble（Ubuntu 22.04 / Py 3.10，解包 `sdk/lib/22/`）及 `/home/ubuntu/xos`。
   - 新增 `./xrobot3 upload`、`./xrobot3 deploy`、`./xrobot3 body [--stop]`、`./xrobot3 remote`、`./xrobot3 interfaces`，启用 SSH `ControlMaster` 免重复输密码；单播 FastDDS XML 配置由 `network.py` 生成。
2. **仿真-真机对齐的力矩上限保护**（`sdk/common/torque_limiter.py`, `sdk/common/robot_interface.py`, `sdk/policy/sdk_a_to/fsm_sdk_a.py`）：
   - 实现 `load_sim_effort_limits`、`clamp_pd_target_by_torque`、`clamp_parallel_ankle_torque`、`enforce_wire_float32_torque_limit`，双层拦截 29 关节 PD 目标、真机 `sptlib` 并联踝与 ROS IDL `float32` 线端误差。
3. **5 状态 FSM + 在线控制台**（`sdk/FSM/robot_fsm.py`, `sdk/common/state_terminal.py`, `sdk/common/websocket_control.py`, `sdk/policy/sdk_a_to/console.html`）：
   - 统一为 `STOP` / `ZERO` / `FIX` / `SDK-A` / `WALK`；新增运行期在线调节 **`FILTER`**（100/50 Hz 一阶低通开关与 `alpha`）与 **`REF`**（参考动作切片 `STAND` / `DANCE` / `FULL`）。
4. **完整交互式 MuJoCo Viewer + 虚拟吊绳**（`sdk/policy/sdk_a_to/render_bridge.py`, `run_chain.py`, `sim_suspension.py`, `scene_unitree_visual.xml`）：
   - 打通原生 `_simulate.Simulate(..., run_physics_thread=True, ...)` 双向控制（`Pause/Run`、`Reset`、`Reload`、4 个内置 `<keyframe>`、关节滑块、小键盘与方向键控吊绳）。
5. **ProtectionStackRuntime 钩子**（`sdk/policy/sdk_a_to/protection_runtime.py`）：可选挂接 B_PLUS / SafeFall / GroundBridge。

### 2. `6209b61b` 之后工作区当前未提交改动（2026-11-12 ~ 11-13）

| 文件 | 本次改动目的与核心内容 |
|---|---|
| `sdk/policy/sdk_a_to/policy.onnx`<br>`sdk/policy/sdk_a_to/manifest.json`<br>`sdk/policy/sdk_a_to/manifest.baseline_30k.json`<br>`sdk/policy/sdk_a_to/policy.baseline_30k.onnx` *(本地备份)* | 将活跃策略升级为 **`model_29618.pt`**（xMimic 正则版 `TO17`，`checkpoint_sha256: fc6b2dbe...`，`onnx_sha256: 96acfc178f09...`，PyTorch/ONNX 误差 `2.38e-6`），原 30k baseline 备份留存。 |
| `sdk/config/sdk_a_sim.yaml`<br>`sdk/config/sdk_a_hardware.yaml`<br>`sdk/local_setup/check_sdk_a_hardware.py` | **控制频率统一对齐为 50 Hz**：`dt` 由 `0.01` 改为 **`0.02`**（外环 50 Hz + `decimation=1` + 策略 50 Hz）；自检脚本放宽并校验 `dt in (0.01, 0.02)`。 |
| `sdk/config/sdk_a_sim.yaml`<br>`sdk/config/sdk_a_hardware.yaml`<br>`sdk/policy/sdk_a_to/fsm_sdk_a.py`<br>`sdk/common/websocket_control.py` | **默认跟随动作安全锁定为 `STAND (50..240)`**：将原危险的默认 `FULL (0..6573)` 全部改为 `STAND`（帧 `50..240`），防止真机切入 `SDK-A` 瞬间直接起跳/跳舞。 |
| `sdk/rl_control_node.py`<br>`sdk/rl_control_node_sim.py` | **修复仿真 `sim=True` 时序竞态**：`XMIGCSControlNode.__init__(debug, sim=...)` 原先忽略 `sim` 参数，`rl_control_node_sim.py` 在控制线程启动后才设置 `robot_interface.sim = True`。现已在加载配置与初始化接口前生效，且 `rl_control_node_sim.py` 默认关联 `sdk_a_sim.yaml`。 |
| `cli.py`<br>`scripts/setup_mujoco4_dev.sh` *(未跟踪)* | **接入 MuJoCo 4.0 实验模式 & GPU 降频警告**：新增 `--physics {stable,dev}`（`dev` 使用 `.venv-mj4` 下的源码编译 MuJoCo 4.0.0）与 `gpu_render_check()` 检测本机 GPU 是否锁死在 30W/Thermal slowdown。 |

> ⚠️ **注意**：上述改动目前均未同步至真机板载机。给用户上真机前，必须提醒执行 `./xrobot3 upload --host ubuntu@<robot-ip>`。

---

## 三、 关键架构决策与硬性不变量（Codex 防错红线）

### 1. 控制频率与滤波器耦合（50 Hz：`dt = 0.02`）
- `sdk_a_sim.yaml` 和 `sdk_a_hardware.yaml` 当前均为 **`dt: 0.02`**、`FSMStateSDKA` 自动算出 `decimation = 1`。
- `sdk/policy/dex_amp/config/dex_amp.yaml` 仍保留 `dt: 0.01, decimation: 2`（`check_sdk_a_hardware.py` 第 125 行有硬断言，切勿改动）。
- **滤波截止频率随 `dt` 变化**：`FSMStateSDKA` 中的一阶 EMA 滤波在每个外环 tick 运行；当 `dt=0.02, alpha=0.35` 时截止频率约为 `4.3 Hz`（而在 `dt=0.01` 时约为 `8.6 Hz`）。`FILTER` 开关和 `alpha` 默认仍为 `false / 0.35`。

### 2. 参考动作切片（`REF`）不破坏 ONNX 契约
- `policy.onnx` 为 **6 输入 / 4 输出** 静态结构（`sensors`, `history`, `previous_action`, `reference_index`, `heading`, `reset`）。**不要重新导出破坏该 IO 契约或改动 `reference.npz` 哈希**。
- `sdk/common/state_terminal.py` 中预设三个切片区间（总长 `0..6573`）：
  - **`STAND`**：`50..240`（**当前默认**，安全原地站立）
  - **`DANCE`**：`250..430`（闭环仿真筛选出的原地律动短片段，适合小场地真机）
  - **`FULL`**：`0..6573`（完整 LAFAN 序列）

### 3. 真机 `body` 与 `remote` 必须双终端分离 + 离线 Ubuntu 24.04 部署约束
- 用户明确要求 **`body_control` 与 `remote` 保持分离**（先前临时耦合脚本已被清理删除）：
  - 终端 1：`./xrobot3 body --host ubuntu@<robot-ip>`（在板载机执行 `chrt -r 99 taskset -c 6-9 ros2 launch body_control body_control.launch.py`，看到 `all was ready` 后保持开启；停止用 `--stop`）。
  - 终端 2：`./xrobot3 remote --host ubuntu@<robot-ip>`（执行 `./xrobot3 run --target real --mode onboard --domain 0`）。
- **离线部署与 ABI 禁忌**：
  - 本机是 Ubuntu 22.04 + Python 3.10 + Humble；板载机是 Ubuntu 24.04 + **Python 3.12** + Jazzy/xos 且**无外网**。
  - `upload` 会跳过 `.venv` 与 `vendor`；切勿将本机的 Python 3.10 `.venv` 直接当作运行环境覆盖给板载机的 Python 3.12（`install.sh` 会自动检测版本并重装 `.venv`、从 `sdk/lib/24/` 解包 `.deb` 和 `cp312` `sptlib_python` wheel）。

### 4. 真机并联踝解算 (`sptlib_python`) 与 `sim` 标志
- **统一入口原则**：`./xrobot3 run --interface lo` 与真机模式都运行同一个 [sdk/rl_control_node.py](file:///home/huan/Xrobot3/sdk/rl_control_node.py)，分别读取 `sdk_a_sim.yaml` (`sim: true`) 和 `sdk_a_hardware.yaml` (`sim: false`)。
- **`sim: false` 真机链路**：开启 `sptlib_python.FuncSPTrans` 并转串 (`ankle_parallel_to_serial`) 与串转并 (`ankle_serial_to_parallel`)，以及 `ct_scale` 电流转换；`robot_interface.py` 在 `get_s_state()` / `get_p_des()` 后强制校验 `success`、`(4,)` 维度和 `np.isfinite`，异常即抛 `RuntimeError` 阻断下发。
- **`rl_control_node.py` 防呆**：如果系统用户名为 `ubuntu` 且 `sim == True`，会直接抛 `RuntimeError` 拒绝启动。
- **切勿在仿真跑 `sim=False`**：MuJoCo 踝关节是串联建模，进入真机并联求解器会越界并打印 `ErrorID00/01 : Cannot solved !`。

### 5. 双层力矩限幅与 `raw/history` 边界
- **不可用限幅/滤波后的目标污染策略历史**：在 [fsm_sdk_a.py](file:///home/huan/Xrobot3/sdk/policy/sdk_a_to/fsm_sdk_a.py#L165-L169) 中，`self.history` 和 `self.raw` 必须保存 **滤波与力矩限幅之前** 的原始策略动作以对齐训练期 `IdealPdActuator`；仅实际下发缓冲 `d.q_d_s_` 经过 `FILTER`（若开启）与 `clamp_pd_target_by_torque`。
- 真机要求 `torque_limit_scale <= 1.0`（代码中有强校验）。
- 实测切入 `SDK-A` 的历史日志（2026-11-11）曾显示首次切入瞬间左肩/左肘与 `FIX` 位姿存在 ~31° 阶跃；但用户明确要求 **不要保留切入平滑（已取消）**，改用可选 `FILTER` 按钮 + 训练端二阶平滑解决。

### 6. MuJoCo Viewer 渲染与防死锁红线（修改 `render_bridge.py` / `run_chain.py` 必读）
1. **绝对禁止在 `with sim.lock():` 内调用 `sim.set_texts(...)`**：C++ `_simulate.so` 的 `set_texts()` 会自旋等待渲染线程清零 `new_texts_`，而渲染线程每帧入口必抢 `sim.lock()`，放在锁内会导致首帧死锁崩溃。
2. **禁止在锁内调用 `sim.load_message` / `sim.load`**。
3. **无锁 VSync 同步与 500 Hz 写入**：`run_chain.py` 每 2 步（`step % 2 == 0`，500 Hz）更新 `viewer_shared.npy` 以消除 100 Hz/60 Hz 拍频卡顿；`render_bridge.py` 用 `data.xfrc_applied[0,0] = 1e-300` 检测渲染帧完成信号，避免与 `RenderLoop` 争锁。
4. **键盘 `-` / `=` 拦截**：已将主键盘 `-` / `=` 与 MuJoCo 调慢 RTF 快捷键解耦（保留吊绳强弱控制绑定 `KP_*`、方向键、`F9`、`Insert`/`Delete` 等）。

### 7. MuJoCo 4.0 `--physics dev` 与本机 RTX 4080 掉帧真因
1. **为何 `--physics dev` 必须全套 GCC 从源码编译**：PyPI 官方 MuJoCo 4.0 仅支持 Python `>= 3.11`，且其预编译 C 库使用 `clang / libc++`。而 ROS Humble 强制绑定 **Python 3.10**，若用 GCC 编绑定去链接官方 `libc++` 库会因 `std::string` ABI 错位在解析 `MjSpec` 时直接段错误，因此必须走 `scripts/setup_mujoco4_dev.sh` 用系统 GCC 同编 C 库与 `mujoco_core`。
2. **本机渲染突然卡在 2 FPS 的硬件固件坑（已踩实）**：
   - 2026-11-13 凌晨排查确认：本机机械革命笔记本（Uniwill/Tongfang `ABBC0F6A` WMI 接口）在**系统休眠（suspend）唤醒后，EC 固件会将 RTX 4080 卡死在 `30W / P8 / 210 MHz` 且 `clocks_event_reasons.sw_thermal_slowdown = Active`**。
   - `nvidia-smi -pl` 不受固件支持，**普通 Reboot 也往往清不掉**。
   - **唯一可靠解法**：**完全关机 -> 拔下电源线等待约 30 秒 -> 插电开机**（即可恢复 `155W`）。`cli.py` 新增的 `gpu_render_check()` 会在启动有界面仿真时自动检测此状态并打印警告。

---

## 四、 模型评测与上游训练现状（给 Codex 的重要更新！）

### 1. 当前 `Xrobot3` 内已部署模型（`model_29618.pt`）实测对比
- 从 `PRO 6000` 拉取并于 2026-11-12 08:00 导出至 `sdk/policy/sdk_a_to/policy.onnx`。
- 其训练挂载了 **`configs/tiangong3_xmimic_regularization_20261111.json`**（SHA256 `c00324a9...`），修复了此前消融脚本误把 `action_rate_l2` 从 `-0.1` 覆写成 `-0.01` 的 10 倍惩罚衰减问题，并新增二阶平滑惩罚 `action_acc_l2: -0.02` 与 PD/摩擦/电枢随机化：
  - **最大 PD 扭矩 / 上限比例**：从原 30k baseline 的 **`1.4701`**（超限 47%）降至 **`0.8327`**（无饱和）
  - **最大动作二阶突变 ($\Delta^2 a$)**：从 **`0.3489`** 降至 **`0.1624`**（**降低 53.4%**）

### 2. 本地磁盘上刚刚跑完的最新模型（尚未导出到 `Xrobot3`！）
2026-11-12 下午，续训至 **35,000 轮** 的两个模型已完整落盘在本机：

| 模型任务 | 本地路径 | 训练设置与关键结论 | 推荐优先级 |
|---|---|---|---|
| **天工3 TO17 35k (`--full`)** | `/media/huan/本地磁盘/stable_mimic/logs/checkpoints/tiangong3_sdk_A_TO17_xmimic_reg_35k_5090/model_34999.pt`<br>(`sha256: d886b1a864c0e580...`) | `5090` 上于 11-12 17:42 跑完 (`completed.json`)，`--full` 纯 FP32，`mirror_branch_matching: false`，带完整 `xmimic_regularization` | **🥇 首选**（若用户要升 35k，优先导出测试此权重） |
| **天工3 TO19 mirror-match 35k** | `/media/huan/本地磁盘/stable_mimic/logs/colab_runs/20261111_tiangong3_sdk_A_TO19_mirror_match_xmimic_full/checkpoints/model_34999.pt`<br>(`sha256: 7ae9ae9c7a13a866...`) | Colab PRO 6000 于 11-12 16:19 跑完 (`completed.json`)，`mirror_branch_matching: true`，含 `onnx_bundle/` | ⚠️ **慎用/仅作对照**：用户在消融中明确否定 `mirror_branch_matching`（认为退化） |

### 3. 上游训练端防踩坑（`stable_mimic`）
- **`--max` 加速存在 KL / LR 锁死 Bug**：2026-11-12 实测 `--max`（BF16 + `torch.compile` PPO）会导致 KL 与学习率卡死在 `1e-5`、reward 停在 `~25.4`。正式训练/续训**一律使用 `--full`**（纯 FP32 + substep CUDA graph，reward 正常维持 `~34+`）。
- **`5090` (32 GiB) 显存控制**：`--constraint-capacity 8192` 会占 ~31 GiB 必爆；须设为 **`--constraint-capacity 2048 --release-cuda-cache-before-rollout`** + `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`。
- **本地无 `stable_mimic/.venv`**：导出 ONNX 前请检查运行环境，若需把服务器导出的 `manifest.json` 放入 `Xrobot3`，记得将绝对路径修正为相对路径（`assets/model29/sdk_a_scene.xml`, `deploy.json`, `reference.npz`）。

---

## 五、 验证基准与下一步建议

### 1. 修改后必跑校验（2026-11-13 最新修改后已全通过）
```bash
# 1. 静态与 50Hz/力矩限幅/ONNX 合约校验
./xrobot3 check

# 2. 交互式仿真（stable=MuJoCo 3.3.7；加 --physics dev 切 MuJoCo 4.0）
./xrobot3 run --interface lo

# 3. 无头 10 秒闭环回归
./xrobot3 run --interface lo --headless --seconds 10
```

### 2. 给 Codex 的紧接下一步建议
1. **确认是否要导出部署 `TO17 35k` (`model_34999.pt`)**：目前 `Xrobot3` 仍是 `model_29618.pt`，而更完整的 35k 权重已落盘；先在 `model_29618` 与 `model_34999` 上做离线动作平滑与扭矩比对，再决定是否更新 `policy.onnx`。
2. **上真机前同步 & 核验**：
   - 执行 `./xrobot3 upload --host ubuntu@<robot-ip>` 将 50Hz 配置、`STAND` 默认切片与新 `policy.onnx` 推上板载机。
   - 双终端分别运行 `./xrobot3 body --host ...` 与 `./xrobot3 remote --host ...`，默认在 `STOP` -> `ZERO` -> `SDK-A`（默认站立 `REF STAND`，安全后再切 `REF DANCE`）。
   - 注意当前 `manifest.json` 仍标记 `"hardware_validated": false`，实机验证时同步确认 IMU 朝向与实机并联踝关节响应，测试通过后再统一整理提交 `Xrobot3` 当前未提交更改。
