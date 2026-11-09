"""SDK-A TO 30k state; exact exported policy sensor/action/history contract."""
from pathlib import Path
import json,numpy as np,onnxruntime as ort
import yaml
from scipy.spatial.transform import Rotation
from FSM.fsm_base import FSMState,FSMStateName

class FSMStateSDKA(FSMState):
    def __init__(self,robot_data):
        super().__init__(robot_data)
        root=Path(__file__).resolve().parent
        self.meta=json.loads((root/'manifest.json').read_text())
        self.contract=json.loads((root/'deploy.json').read_text())
        config_path=getattr(robot_data,'config_file_','')
        cfg=yaml.safe_load(Path(config_path).read_text()) if config_path else {}
        self.hardware=cfg.get('sdk_a_hardware',False)
        self.settings=cfg.get('sdk_a',{})
        if self.hardware and self.settings.get('imu_input_frame') != 'sdk_body_euler' and not self.settings.get('imu_mount_verified',False):
            raise ValueError('Set the measured IMU-to-pelvis mounting quaternion and verify it before hardware deployment')
        opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(root/'policy.onnx'),opt,providers=['CPUExecutionProvider'])
        self.on_enter()
    def on_enter(self):
        self.history=np.zeros((1,884),np.float32);self.raw=np.zeros((1,29),np.float32);self.reset=True
        self.counter=0;self.phase=0;self.running=self.hardware;self.heading=np.array([[1,0,0,0]],np.float32);self.weights=np.array([1.,0.])
        self.robot_data_.joint_kp_s_[:]=self.contract['kp'];self.robot_data_.joint_kd_s_[:]=self.contract['kd']
    def run(self,flag):
        d=self.robot_data_
        if self.hardware:
            if self.settings.get('imu_input_frame') == 'sdk_body_euler':
                # Match upstream walk_amp exactly: roll, pitch, yaw; body-frame gyro.
                pelvis=Rotation.from_euler('xyz',d.imu_data_[[2,1,0]])
                d.sdk_a_gyro=np.array(d.imu_data_[3:6],copy=True)
            else:
                mount=Rotation.from_quat(self.settings['imu_mount_quat_wxyz'],scalar_first=True)
                pelvis=Rotation.from_quat(d.sdk_a_imu_quat,scalar_first=True)*mount.inv()
                d.sdk_a_gyro=mount.apply(d.imu_data_[3:6])
            d.sdk_a_pelvis_quat=pelvis.as_quat(scalar_first=True)
            if self.reset:
                yaw=pelvis.as_euler('xyz')[2]
                self.heading=Rotation.from_euler('z',yaw).as_quat(scalar_first=True)[None].astype(np.float32)
        sensors=np.r_[d.q_a_s_[6:],d.q_dot_a_s_[6:],d.sdk_a_pelvis_quat,d.sdk_a_gyro][None].astype(np.float32)
        feed=dict(sensors=sensors,history=self.history,previous_action=self.raw,
          reference_index=np.array([self.phase+self.counter],np.int64),heading=self.heading,reset=np.array([[self.reset]],bool))
        target,raw,hist,weights=self.session.run(None,feed)
        if not all(np.isfinite(x).all() for x in (target,raw,hist,weights)):raise RuntimeError('SDK-A nonfinite output')
        d.q_d_s_[6:]=target[0];d.q_dot_d_s_.fill(0);d.tau_d_s_.fill(0)
        self.target=target[0];self.weights=weights[0];self.output_raw=raw[0]
        if self.running:self.counter+=1;self.reset=False;self.history=hist;self.raw=raw
    def on_exit(self):pass
    def check_transition(self,flag):
        return {'gotoDEXAMP':FSMStateName.DEXAMP,'gotoSTOP':FSMStateName.STOP,'gotoZERO':FSMStateName.ZERO}.get(flag.fsm_state_command,FSMStateName.SDK_A)
