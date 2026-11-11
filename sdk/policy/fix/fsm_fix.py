"""Fixed-standing FSM: interpolate once, then hold; no simulation assistance."""
import json
from pathlib import Path
import numpy as np
from FSM.fsm_base import FSMState,FSMStateName

class FSMStateFix(FSMState):
    def __init__(self,robot_data):
        super().__init__(robot_data)
        c=json.loads(Path(__file__).with_name('config.json').read_text())
        self.current_state_name=FSMStateName.FIX
        self.target=np.asarray(c['target'],dtype=float)
        self.kp=np.asarray(c['kp'],dtype=float);self.kd=np.asarray(c['kd'],dtype=float)
        self.duration=float(c['duration_s'])
        if self.duration<=0 or any(x.shape!=(29,) or not np.isfinite(x).all() for x in (self.target,self.kp,self.kd)):
            raise ValueError('Invalid FIX joint targets/gains/duration')
    def on_enter(self):
        qa=self.robot_data_.q_a_s_[-29:]
        self.start=(qa if np.any(qa) else self.robot_data_.q_d_s_[-29:]).copy()
        self.elapsed=0.0
        print('[FIX] Interpolating to configured standing pose')
    def run(self,flag):
        blend=min(self.elapsed/self.duration,1.0)
        self.robot_data_.q_d_s_[-29:]=(1-blend)*self.start+blend*self.target
        self.robot_data_.q_dot_d_s_[-29:]=0
        self.robot_data_.tau_d_s_[-29:]=0
        self.robot_data_.joint_kp_s_[:29]=self.kp
        self.robot_data_.joint_kd_s_[:29]=self.kd
        self.elapsed+=self.robot_data_.control_dt_
    def on_exit(self):pass
    def check_transition(self,flag):
        return {'gotoSTOP':FSMStateName.STOP,'gotoZERO':FSMStateName.ZERO,
                'gotoSDK_A':FSMStateName.SDK_A,'gotoDEXAMP':FSMStateName.DEXAMP,
                'gotoFIX':FSMStateName.FIX}.get(flag.fsm_state_command)
