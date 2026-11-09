import mujoco

model = mujoco.MjModel.from_xml_path("xsim_mujoco/urdf/tiangong2dex_evt_mujoco.urdf")

mujoco.mj_saveLastXML("/home/eai/Dev/dex_ankle/xsim_mujoco/urdf/dex_evt.xml",model)