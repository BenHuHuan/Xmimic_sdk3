# visualize_dex0902.py
import mujoco
import mujoco.viewer
import time
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path(
    '/home/eai/Dev/dex_ankle/xsim_mujoco/urdf/dex_evt.xml')
# model = mujoco.MjModel.from_xml_path('/home/eai/Dev/dex_ankle/dex0902/urdf/dex0902.xml')

data = mujoco.MjData(model)

# 启动可视化器
viewer = mujoco.viewer.launch_passive(model, data)

# 运行仿真
while viewer.is_running():
    step_start = time.time()

    # 前进仿真一步
    mujoco.mj_step(model, data)

    # 更新可视化
    viewer.sync()

    # 控制仿真速度
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

# 关闭可视化器
viewer.close()
