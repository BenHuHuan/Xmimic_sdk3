"""
MuJoCo机器人控制脚本
实现位置和力矩混合控制，读取传感器数据
"""
import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


class RobotController:

    def __init__(self, model_path):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 初始化控制模式 (0=位置控制, 1=力矩控制)
        self.control_mode = 0

        # 记录数据用于绘图
        self.time_history = deque(maxlen=1000)
        self.joint_pos_history = deque(maxlen=1000)
        self.joint_vel_history = deque(maxlen=1000)
        self.joint_torque_history = deque(maxlen=1000)
        self.imu_orient_history = deque(maxlen=1000)
        self.imu_accel_history = deque(maxlen=1000)
        self.imu_pos_history = deque(maxlen=1000)

        # 获取关节索引
        self.joint_names = [
            'hip_pitch_l_joint', 'hip_roll_l_joint', 'hip_yaw_l_joint',
            'knee_pitch_l_joint', 'ankle_pitch_l_joint', 'ankle_roll_l_joint',
            'hip_pitch_r_joint', 'hip_roll_r_joint', 'hip_yaw_r_joint',
            'knee_pitch_r_joint', 'ankle_pitch_r_joint', 'ankle_roll_r_joint',
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            'shoulder_pitch_l_joint', 'shoulder_roll_l_joint',
            'shoulder_yaw_l_joint', 'elbow_pitch_l_joint', 'elbow_yaw_l_joint',
            'wrist_pitch_l_joint', 'wrist_roll_l_joint',
            'shoulder_pitch_r_joint', 'shoulder_roll_r_joint',
            'shoulder_yaw_r_joint', 'elbow_pitch_r_joint', 'elbow_yaw_r_joint',
            'wrist_pitch_r_joint', 'wrist_roll_r_joint'
        ]

        self.joint_indices = []
        for name in self.joint_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                    name)
            self.joint_indices.append(idx)

        # 获取执行器索引
        self.pos_actuator_names = [f"pos_{name}" for name in self.joint_names]
        self.motor_actuator_names = [
            f"motor_{name}" for name in self.joint_names
        ]

        self.pos_actuator_indices = []
        self.motor_actuator_indices = []

        for name in self.pos_actuator_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    name)
            self.pos_actuator_indices.append(idx)

        for name in self.motor_actuator_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    name)
            self.motor_actuator_indices.append(idx)

        # 获取传感器索引
        self.joint_pos_sensor_names = [
            f"{name}_pos" for name in self.joint_names
        ]
        self.joint_vel_sensor_names = [
            f"{name}_vel" for name in self.joint_names
        ]
        self.joint_torque_sensor_names = [
            f"{name}_torque" for name in self.joint_names
        ]

        self.joint_pos_sensor_indices = []
        self.joint_vel_sensor_indices = []
        self.joint_torque_sensor_indices = []

        for name in self.joint_pos_sensor_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR,
                                    name)
            self.joint_pos_sensor_indices.append(idx)

        for name in self.joint_vel_sensor_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR,
                                    name)
            self.joint_vel_sensor_indices.append(idx)

        for name in self.joint_torque_sensor_names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR,
                                    name)
            self.joint_torque_sensor_indices.append(idx)

        # IMU传感器索引
        self.imu_orient_idx = mujoco.mj_name2id(self.model,
                                                mujoco.mjtObj.mjOBJ_SENSOR,
                                                'orientation')
        self.imu_pos_idx = mujoco.mj_name2id(self.model,
                                             mujoco.mjtObj.mjOBJ_SENSOR,
                                             'position')
        self.imu_gyro_idx = mujoco.mj_name2id(self.model,
                                              mujoco.mjtObj.mjOBJ_SENSOR,
                                              'angular-velocity')
        self.imu_vel_idx = mujoco.mj_name2id(self.model,
                                             mujoco.mjtObj.mjOBJ_SENSOR,
                                             'linear-velocity')
        self.imu_accel_idx = mujoco.mj_name2id(self.model,
                                               mujoco.mjtObj.mjOBJ_SENSOR,
                                               'linear-acceleration')
        self.imu_mag_idx = mujoco.mj_name2id(self.model,
                                             mujoco.mjtObj.mjOBJ_SENSOR,
                                             'magnetometer')

    def read_sensors(self):
        """读取所有传感器数据"""
        # 读取关节传感器数据
        joint_positions = []
        joint_velocities = []
        joint_torques = []

        for i in range(len(self.joint_names)):
            pos_idx = self.joint_pos_sensor_indices[i]
            vel_idx = self.joint_vel_sensor_indices[i]
            torque_idx = self.joint_torque_sensor_indices[i]

            joint_positions.append(self.data.sensordata[pos_idx])
            joint_velocities.append(self.data.sensordata[vel_idx])
            joint_torques.append(self.data.sensordata[torque_idx])

        # 读取IMU数据
        imu_orientation = self.data.sensordata[self.imu_orient_idx:self.
                                               imu_orient_idx + 4]
        imu_position = self.data.sensordata[self.imu_pos_idx:self.imu_pos_idx +
                                            3]
        imu_angular_velocity = self.data.sensordata[self.imu_gyro_idx:self.
                                                    imu_gyro_idx + 3]
        imu_linear_velocity = self.data.sensordata[self.imu_vel_idx:self.
                                                   imu_vel_idx + 3]
        imu_linear_acceleration = self.data.sensordata[self.imu_accel_idx:self.
                                                       imu_accel_idx + 3]
        imu_magnetometer = self.data.sensordata[self.
                                                imu_mag_idx:self.imu_mag_idx +
                                                3]

        return {
            'joint_positions': np.array(joint_positions),
            'joint_velocities': np.array(joint_velocities),
            'joint_torques': np.array(joint_torques),
            'imu_orientation': imu_orientation,
            'imu_position': imu_position,
            'imu_angular_velocity': imu_angular_velocity,
            'imu_linear_velocity': imu_linear_velocity,
            'imu_linear_acceleration': imu_linear_acceleration,
            'imu_magnetometer': imu_magnetometer
        }

    def set_position_control(self, target_positions):
        """设置位置控制"""
        # 禁用力矩执行器
        for i in range(len(self.motor_actuator_indices)):
            if self.motor_actuator_indices[i] != -1:
                self.data.ctrl[self.motor_actuator_indices[i]] = 0

        # 启用位置执行器
        for i in range(len(self.pos_actuator_indices)):
            if self.pos_actuator_indices[i] != -1:
                self.data.ctrl[
                    self.pos_actuator_indices[i]] = target_positions[i]

    def set_torque_control(self, target_torques):
        """设置力矩控制"""
        # 禁用位置执行器
        for i in range(len(self.pos_actuator_indices)):
            if self.pos_actuator_indices[i] != -1:
                self.data.ctrl[self.pos_actuator_indices[i]] = 0

        # 启用力矩执行器
        for i in range(len(self.motor_actuator_indices)):
            if self.motor_actuator_indices[i] != -1:
                self.data.ctrl[
                    self.motor_actuator_indices[i]] = target_torques[i]

    def run_control_loop(self):
        """运行控制循环"""
        # 启动可视化器
        viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # 控制参数
        time_step = 0
        control_frequency = 100  # 100Hz控制频率
        sensor_frequency = 100  # 100Hz传感器读取频率
        control_interval = 1 / control_frequency

        # 目标轨迹
        target_time = 0
        last_control_time = 0
        try:
            while viewer.is_running():
                step_start = time.time()

                # 更新目标轨迹
                target_time += self.model.opt.timestep
                target_positions = np.sin(target_time * 2) * 0.3  # 简单的正弦轨迹

                if target_time - last_control_time >= control_interval:
                    # 切换控制模式 (每5秒切换一次)
                    if int(target_time) % 10 < 2:
                        self.control_mode = 0  # 位置控制
                        # 为所有关节设置相同的目标位置
                        targets = np.full(len(self.joint_names),
                                          target_positions)
                        self.set_position_control(targets)
                    else:
                        self.control_mode = 1  # 力矩控制
                        # 为所有关节设置相同的力矩
                        torques = np.full(len(self.joint_names),
                                          target_positions * 10)
                        self.set_torque_control(torques)

                # 前进仿真一步
                mujoco.mj_step(self.model, self.data)

                # 读取传感器数据
                if int(1.0 / self.model.opt.timestep * target_time) % int(
                        1.0 / self.model.opt.timestep / sensor_frequency) == 0:
                    sensor_data = self.read_sensors()

                    # 存储数据用于绘图
                    self.time_history.append(target_time)
                    self.joint_pos_history.append(
                        sensor_data['joint_positions'].copy())
                    self.joint_vel_history.append(
                        sensor_data['joint_velocities'].copy())
                    self.joint_torque_history.append(
                        sensor_data['joint_torques'].copy())
                    self.imu_orient_history.append(
                        sensor_data['imu_orientation'].copy())
                    self.imu_accel_history.append(
                        sensor_data['imu_linear_acceleration'].copy())
                    self.imu_pos_history.append(
                        sensor_data['imu_position'].copy())
                    # 打印部分传感器数据
                    if time_step % 100 == 0:
                        print(f"Time: {target_time:.3f}s")
                        print(
                            f"Control Mode: {'Position' if self.control_mode == 0 else 'Torque'}"
                        )
                        print(
                            f"Joint 0 Pos: {sensor_data['joint_positions'][0]:.3f}, "
                            f"Vel: {sensor_data['joint_velocities'][0]:.3f}, "
                            f"Torque: {sensor_data['joint_torques'][0]:.3f}")
                        print(
                            f"IMU Orientation: [{sensor_data['imu_orientation'][0]:.3f}, "
                            f"{sensor_data['imu_orientation'][1]:.3f}, "
                            f"{sensor_data['imu_orientation'][2]:.3f}, "
                            f"{sensor_data['imu_orientation'][3]:.3f}]")
                        print(
                            f"IMU Acceleration: [{sensor_data['imu_linear_acceleration'][0]:.3f}, "
                            f"{sensor_data['imu_linear_acceleration'][1]:.3f}, "
                            f"{sensor_data['imu_linear_acceleration'][2]:.3f}]"
                        )
                        print("-" * 50)

                # 更新可视化
                viewer.sync()

                time_step += 1

                # 控制仿真速度
                time_until_next_step = self.model.opt.timestep - (time.time() -
                                                                  step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            viewer.close()
            self.plot_sensor_data()

    def plot_sensor_data(self):
        """绘制传感器数据"""
        if len(self.time_history) == 0:
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Sensor Data')

        time_array = np.array(list(self.time_history))

        # 绘制关节位置
        if len(self.joint_pos_history) > 0:
            pos_array = np.array(list(self.joint_pos_history))
            axes[0, 0].plot(time_array, pos_array[:, 0], label='Joint 0')
            axes[0, 0].plot(time_array, pos_array[:, 1], label='Joint 1')
            axes[0, 0].set_title('Joint Positions')
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Position (rad)')
            axes[0, 0].legend()
            axes[0, 0].grid(True)

        # 绘制关节速度
        if len(self.joint_vel_history) > 0:
            vel_array = np.array(list(self.joint_vel_history))
            axes[0, 1].plot(time_array, vel_array[:, 0], label='Joint 0')
            axes[0, 1].plot(time_array, vel_array[:, 1], label='Joint 1')
            axes[0, 1].set_title('Joint Velocities')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Velocity (rad/s)')
            axes[0, 1].legend()
            axes[0, 1].grid(True)

        # 绘制IMU方向
        if len(self.imu_orient_history) > 0:
            orient_array = np.array(list(self.imu_orient_history))
            axes[1, 0].plot(time_array, orient_array[:, 0], label='w')
            axes[1, 0].plot(time_array, orient_array[:, 1], label='x')
            axes[1, 0].plot(time_array, orient_array[:, 2], label='y')
            axes[1, 0].plot(time_array, orient_array[:, 3], label='z')
            axes[1, 0].set_title('IMU Orientation (Quaternion)')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Quaternion')
            axes[1, 0].legend()
            axes[1, 0].grid(True)

        # # 绘制IMU加速度
        # if len(self.imu_accel_history) > 0:
        #     accel_array = np.array(list(self.imu_accel_history))
        #     axes[1, 1].plot(time_array, accel_array[:, 0], label='X')
        #     axes[1, 1].plot(time_array, accel_array[:, 1], label='Y')
        #     axes[1, 1].plot(time_array, accel_array[:, 2], label='Z')
        #     axes[1, 1].set_title('IMU Linear Acceleration')
        #     axes[1, 1].set_xlabel('Time (s)')
        #     axes[1, 1].set_ylabel('Acceleration (m/s²)')
        #     axes[1, 1].legend()
        #     axes[1, 1].grid(True)

        # 绘制IMU位置
        if len(self.imu_pos_history) > 0:
            pos_array = np.array(list(self.imu_pos_history))
            axes[1, 1].plot(time_array, pos_array[:, 0], label='X')
            axes[1, 1].plot(time_array, pos_array[:, 1], label='Y')
            axes[1, 1].plot(time_array, pos_array[:, 2], label='Z')
            axes[1, 1].set_title('IMU Linear Position')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Position (m)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()
    
def main():
    # 设置模型路径
    model_path = '/home/eai/Dev/dex_ankle/dex0902/urdf/dex0902_mujoco.xml'

    # 创建控制器
    controller = RobotController(model_path)

    # 运行控制循环
    controller.run_control_loop()

if __name__ == "__main__":
    main()
