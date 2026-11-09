import mujoco
import math

# 1. 创建一个简单的XML模型字符串，描述一个球体
xml_string = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="ball" pos="0 0 1">
      <joint type="free"/> <!-- 自由关节，允许球体自由运动 -->
      <geom name="ball_geom" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
"""

# 2. 从XML字符串加载模型
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# 3. 打印初始状态（球的位置）
print("初始状态 - 球的位置Z坐标:", data.qpos[2])

# 4. 运行一步模拟（让重力起作用）
mujoco.mj_step(model, data)

# 5. 打印模拟后状态，观察位置变化
print("模拟一步后 - 球的位置Z坐标:", data.qpos[2])