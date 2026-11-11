"""SDK-A TO 30k state; exact exported policy sensor/action/history contract."""
from pathlib import Path
import json,os,sys,numpy as np,onnxruntime as ort
import yaml
from scipy.spatial.transform import Rotation
from FSM.fsm_base import FSMState,FSMStateName
from common.control_timing import TickDecimator
from common.torque_limiter import clamp_pd_target_by_torque

class FSMStateSDKA(FSMState):
    def __init__(self,robot_data):
        super().__init__(robot_data)
        root=Path(__file__).resolve().parent
        if str(root) not in sys.path:sys.path.insert(0,str(root))
        self.meta=json.loads((root/'manifest.json').read_text())
        self.contract=json.loads((root/'deploy.json').read_text())
        self.kp=np.asarray(self.contract['kp'],dtype=np.float64)
        self.kd=np.asarray(self.contract['kd'],dtype=np.float64)
        self.effort=np.asarray(self.contract['effort'],dtype=np.float64)
        if any(x.shape!=(29,) or not np.isfinite(x).all() for x in (self.kp,self.kd,self.effort)) or np.any(self.kp<=0) or np.any(self.kd<0) or np.any(self.effort<=0):
            raise ValueError('Invalid 29-DOF kp/kd/effort in SDK-A deploy.json')
        config_path=getattr(robot_data,'config_file_','')
        cfg=yaml.safe_load(Path(config_path).read_text()) if config_path else {}
        self.hardware=cfg.get('sdk_a_hardware',False)
        self.settings=dict(cfg.get('sdk_a',{}))
        self.ref_name=self.settings.get('ref_name','FULL')
        self.torque_limit_enabled=bool(self.settings.get('torque_limit_enabled',True))
        self.torque_limit_scale=float(self.settings.get('torque_limit_scale',1.0))
        if not np.isfinite(self.torque_limit_scale) or self.torque_limit_scale<=0.0 or (self.hardware and self.torque_limit_scale>1.0):
            raise ValueError(f'Invalid sdk_a.torque_limit_scale={self.torque_limit_scale}')
        self.torque_limits=self.effort*self.torque_limit_scale
        self.torque_limit_telemetry=None
        self.action_filter_enabled=bool(self.settings.get('action_filter_enabled',False))
        self.action_filter_alpha=float(self.settings.get('action_filter_alpha',0.35))
        if not np.isfinite(self.action_filter_alpha) or not (0.0<self.action_filter_alpha<=1.0):
            raise ValueError(f'Invalid sdk_a.action_filter_alpha={self.action_filter_alpha}; expected (0, 1]')
        self.policy_dt=float(self.contract.get('policy_dt',0.02))
        control_dt=getattr(robot_data,'control_dt_',None)
        if control_dt is None:
            control_dt=float(cfg.get('dt',0.01))
        else:
            control_dt=float(control_dt)
        decimation_cfg=self.settings.get('decimation')
        if decimation_cfg is None:
            ratio=self.policy_dt/control_dt
            decimation=max(1,int(round(ratio)))
        else:
            decimation=int(decimation_cfg)
        if not np.isclose(decimation*control_dt,self.policy_dt,rtol=1e-9,atol=1e-12):
            raise ValueError(f'SDK-A requires {1.0/self.policy_dt:g}Hz inference: decimation={decimation} * control_dt={control_dt} != {self.policy_dt}')
        self.control_dt=control_dt
        self.decimation=decimation
        self._inference_clock=TickDecimator(self.decimation,self.control_dt,getattr(robot_data,'control_dt_',None))
        self.unclamped_target=np.asarray(self.contract['offset'],dtype=np.float64).copy()
        self._filtered_target=self.unclamped_target.copy()
        self.output_raw=np.zeros(29,dtype=np.float32)
        self.target=self.unclamped_target.astype(np.float32)
        if self.hardware and self.settings.get('imu_input_frame') != 'sdk_body_euler' and not self.settings.get('imu_mount_verified',False):
            raise ValueError('Set the measured IMU-to-pelvis mounting quaternion and verify it before hardware deployment')
        opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(root/'policy.onnx'),opt,providers=['CPUExecutionProvider'])
        bplus_dir=self.settings.get('bplus_bundle') or os.environ.get('XROBOT_BPLUS_BUNDLE')
        bridge_dir=self.settings.get('ground_bridge_bundle') or os.environ.get('XROBOT_GROUND_BRIDGE_BUNDLE')
        if bridge_dir and not bplus_dir:
            raise ValueError('GroundBridge requires B_PLUS protection bundle')
        self.protection=None
        self.protection_telemetry=None
        if bplus_dir:
            from protection_runtime import ProtectionStackRuntime
            clamp_ankles=bool(self.settings.get('clamp_parallel_ankles',self.hardware))
            self.protection=ProtectionStackRuntime(
                self.contract,
                bplus_bundle=bplus_dir,
                ground_bridge_bundle=bridge_dir,
                clamp_parallel_ankles=clamp_ankles,
            )
        self.on_enter()
    def set_action_filter(self,enabled=None,alpha=None):
        if alpha is not None:
            alpha=float(alpha)
            if not np.isfinite(alpha) or not (0.0<alpha<=1.0):
                raise ValueError('Filter alpha must satisfy 0 < alpha <= 1.0')
            self.action_filter_alpha=alpha
            if alpha>=1.0 and enabled is None:
                enabled=False
        if enabled is None:
            self.action_filter_enabled=not self.action_filter_enabled
        else:
            self.action_filter_enabled=bool(enabled)
        self.settings['action_filter_enabled']=self.action_filter_enabled
        self.settings['action_filter_alpha']=self.action_filter_alpha
        return self.action_filter_enabled,self.action_filter_alpha
    def action_filter_cutoff_hz(self):
        if not self.action_filter_enabled or self.action_filter_alpha>=1.0:
            return None
        tau=self.control_dt*(1.0-self.action_filter_alpha)/self.action_filter_alpha
        return float(1.0/(2.0*np.pi*max(tau,1e-9)))
    def set_reference_window(self,start,end,name=None):
        start=int(start);end=int(end)
        if start<0 or end<start or end>=6574:raise ValueError('Reference frame range must satisfy 0 <= start <= end < 6574')
        self.settings['ref_start']=start;self.settings['ref_end']=end
        self.ref_name=name or f'{start}..{end}'
        self.counter=0
    def on_enter(self):
        self._inference_clock.reset()
        self.history=np.zeros((1,884),np.float32);self.raw=np.zeros((1,29),np.float32);self.reset=True
        self.counter=0;self.phase=0;self.running=self.hardware;self.heading=np.array([[1,0,0,0]],np.float32);self.weights=np.array([1.,0.])
        qa=self.robot_data_.q_a_s_[6:]
        init_pose=np.asarray(qa if np.any(qa) else self.contract['offset'],dtype=np.float64).copy()
        self.unclamped_target=np.asarray(self.contract['offset'],dtype=np.float64).copy()
        self._filtered_target=init_pose
        self.output_raw=np.zeros(29,dtype=np.float32)
        self.target=init_pose.astype(np.float32)
        self.robot_data_.joint_kp_s_[:]=self.kp;self.robot_data_.joint_kd_s_[:]=self.kd
        self.torque_limit_telemetry=None
        if self.protection is not None:
            self.protection.reset()
            self.protection_telemetry=dict(self.protection.last_telemetry)
    def run(self,flag):
        d=self.robot_data_
        if self._inference_clock.step():
            if self.hardware:
                if self.settings.get('imu_input_frame') == 'sdk_body_euler':
                    if self.settings.get('prefer_imu_quat',False) and hasattr(d,'sdk_a_imu_quat') and np.linalg.norm(d.sdk_a_imu_quat)>0.9:
                        pelvis=Rotation.from_quat(d.sdk_a_imu_quat,scalar_first=True)
                    else:
                        # Match upstream walk_amp exactly: roll, pitch, yaw; body-frame gyro.
                        pelvis=Rotation.from_euler('xyz',d.imu_data_[[2,1,0]])
                    d.sdk_a_gyro=np.array(d.imu_data_[3:6],copy=True)
                    d.sdk_a_acc=np.array(d.imu_data_[6:9],copy=True)
                else:
                    mount=Rotation.from_quat(self.settings['imu_mount_quat_wxyz'],scalar_first=True)
                    pelvis=Rotation.from_quat(d.sdk_a_imu_quat,scalar_first=True)*mount.inv()
                    d.sdk_a_gyro=mount.apply(d.imu_data_[3:6])
                    d.sdk_a_acc=mount.apply(d.imu_data_[6:9])
                d.sdk_a_pelvis_quat=pelvis.as_quat(scalar_first=True)
                if self.reset:
                    yaw=pelvis.as_euler('xyz')[2]
                    self.heading=Rotation.from_euler('z',yaw).as_quat(scalar_first=True)[None].astype(np.float32)
            sensors=np.r_[d.q_a_s_[6:],d.q_dot_a_s_[6:],d.sdk_a_pelvis_quat,d.sdk_a_gyro][None].astype(np.float32)
            ref_start=int(self.settings.get('ref_start',0))
            ref_end=int(self.settings.get('ref_end',6573))
            ref_span=max(1,ref_end-ref_start+1)
            ref_idx=ref_start+((self.phase+self.counter)%ref_span)
            prev_executed_raw=self.raw[0].copy()
            feed=dict(sensors=sensors,history=self.history,previous_action=self.raw,
              reference_index=np.array([ref_idx],np.int64),heading=self.heading,reset=np.array([[self.reset]],bool))
            target,raw,hist,weights=self.session.run(None,feed)
            if not all(np.isfinite(x).all() for x in (target,raw,hist,weights)):raise RuntimeError('SDK-A nonfinite output')
            if self.protection is not None and self.running:
                pelvis_acc=np.asarray(getattr(d,'sdk_a_acc',np.array([0.,0.,9.81],np.float32)),dtype=np.float32)
                exec_target,exec_raw,tele=self.protection.step(
                    sensors_65=sensors[0],
                    pelvis_acc_3=pelvis_acc,
                    sdk_target_29=target[0],
                    sdk_raw_29=raw[0],
                    sdk_history_884=hist[0],
                    previous_executed_raw_29=prev_executed_raw,
                )
                target=exec_target[None]
                raw=exec_raw[None]
                self.protection_telemetry=tele
            self.unclamped_target=np.asarray(target[0],dtype=np.float64).copy()
            self.weights=weights[0]
            self.output_raw=raw[0].copy()
            # Keep raw/history before actuator filtering and torque saturation to match training IdealPdActuator.
            if self.running:
                self.counter+=1;self.reset=False;self.history=hist;self.raw=raw
        if self.action_filter_enabled and self.action_filter_alpha<1.0:
            self._filtered_target+=self.action_filter_alpha*(self.unclamped_target-self._filtered_target)
            desired_target=self._filtered_target.copy()
        else:
            self._filtered_target[:]=self.unclamped_target
            desired_target=self.unclamped_target
        if self.torque_limit_enabled:
            exec_q_d,_,tau_raw,tau_clip,clipped_mask=clamp_pd_target_by_torque(
                q_d=desired_target,
                q_a=d.q_a_s_[6:],
                q_dot_a=d.q_dot_a_s_[6:],
                kp=self.kp,
                kd=self.kd,
                effort_limit=self.torque_limits,
            )
            self.torque_limit_telemetry=dict(
                enabled=True,
                scale=self.torque_limit_scale,
                clipped_joints=int(clipped_mask.sum()),
                max_raw_effort_ratio=float(np.max(np.abs(tau_raw)/self.torque_limits)),
                max_cmd_effort_ratio=float(np.max(np.abs(tau_clip)/self.torque_limits)),
            )
        else:
            exec_q_d=desired_target
            self.torque_limit_telemetry=dict(enabled=False,scale=self.torque_limit_scale,clipped_joints=0)
        d.q_d_s_[6:]=exec_q_d;d.q_dot_d_s_.fill(0);d.tau_d_s_.fill(0)
        d.joint_kp_s_[:]=self.kp;d.joint_kd_s_[:]=self.kd
        self.target=np.asarray(exec_q_d,dtype=np.float32)
    def on_exit(self):pass
    def check_transition(self,flag):
        return {'gotoDEXAMP':FSMStateName.DEXAMP,'gotoSTOP':FSMStateName.STOP,'gotoZERO':FSMStateName.ZERO}.get(flag.fsm_state_command,FSMStateName.SDK_A)
