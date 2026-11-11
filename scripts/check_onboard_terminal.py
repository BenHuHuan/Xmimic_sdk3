"""Offline onboard command/FSM test: no ROS nodes or motor/network publishers."""
from pathlib import Path
import os,sys,queue,io
from unittest.mock import patch
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sdk'))
from common.robot_data import RobotData
from common.state_terminal import parse_terminal_command,start_state_terminal
from common.websocket_control import WebsocketController
from FSM.robot_fsm import RobotFSMImpl

cfgpath=ROOT/'sdk/config/sdk_a_hardware.yaml'
cfg=yaml.safe_load(cfgpath.read_text())
assert not cfg['sim'] and cfg['sdk_a_only'] and not cfg['sdk_a_autostart']
rd=RobotData(29,35);rd.config_file_=str(cfgpath);rd.control_dt_=cfg['dt']
fsm=RobotFSMImpl(rd,cfg)
ctrl=WebsocketController(rd,fsm)  # Constructor only: do not call init()/start().
assert ctrl.max_x_plus_speed==cfg['websocket']['max_x_plus_speed']
assert not hasattr(ctrl,'simulation_fix')
states={'STOP':'STOP','ZERO':'ZERO','FIX':'FIX','SDK-A':'SDK_A','WALK':'DEXAMP'}
def execute(text):
 event,value=parse_terminal_command(text)
 reply=queue.Queue()
 ctrl._apply_event('terminal_request',(event,value,reply))
 fsm.run_fsm(ctrl.flag)
 ctrl.complete_terminal_requests()
 result=reply.get(timeout=1)
 assert np.isfinite(rd.q_d_s_).all(),text
 return result
for source in states:
 for dest,expected in states.items():
  execute(source)
  answer=execute(dest)
  assert fsm.get_current_state().name==expected,(source,dest,answer)
execute('WALK')
walk=fsm.state_objects[fsm.get_current_state()]
assert 'Velocity applied' in execute('VEL 0.3 0')
assert ctrl.flag.x_speed_command==.3 and ctrl.flag.yaw_speed_command==0
assert fsm.state_objects[fsm.get_current_state()] is walk
execute('FIX')
assert ctrl.flag.x_speed_command==ctrl.flag.y_speed_command==ctrl.flag.yaw_speed_command==0
assert 'Not executed' in execute('VEL 0.3 0')
assert fsm.get_current_state().name=='FIX'
execute('SDK-A')
sdka_obj=fsm.state_objects[fsm.get_current_state()]
assert sdka_obj.ref_name=='FULL' and sdka_obj.settings.get('ref_start')==0 and sdka_obj.settings.get('ref_end')==6573
assert 'DANCE (frames 250..430' in execute('REF DANCE')
assert '304..430 (frames 304..430' in execute('REF 304 430')
assert 'STAND (frames 50..240' in execute('REF STAND')
assert 'FULL (frames 0..6573' in execute('REF FULL')
assert 'ON (alpha=0.35' in execute('FILTER ON')
assert 'ON (alpha=0.25' in execute('FILTER STRONG')
assert 'ON (alpha=0.4' in execute('FILTER 0.4')
assert 'OFF' in execute('FILTER')
assert 'ON (alpha=0.4' in execute('FILTER')
assert 'OFF' in execute('FILTER OFF')
assert ctrl._apply_event('filter_toggle', 2) and fsm.state_objects[fsm.get_current_state()].action_filter_enabled
assert ctrl._apply_event('filter_off', 2) and not fsm.state_objects[fsm.get_current_state()].action_filter_enabled
# Use the actual stdin parser in hardware mode: HANG is rejected locally.
out=io.StringIO()
with patch.dict(os.environ,{},clear=True),patch('sys.stdin',io.StringIO('HANG ON\n')),patch('sys.stdout',out):
 thread=start_state_terminal(ctrl);thread.join(timeout=2)
 assert not thread.is_alive()
assert 'HANG is available only in simulation.' in out.getvalue()
print('PASS: hardware-config 25 state transitions, WALK velocity, clearing/rejection, HANG exclusion; no hardware I/O.')
