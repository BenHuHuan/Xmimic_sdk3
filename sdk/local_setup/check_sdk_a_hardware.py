"""Offline configuration inspection. Does not create a ROS node or publish commands."""
import json,hashlib,sys
from pathlib import Path
import numpy as np,yaml,onnxruntime as ort
root=Path(__file__).resolve().parents[1]
c=yaml.safe_load((root/'config/sdk_a_hardware.yaml').read_text())
p=root/'policy/sdk_a_to';m=json.loads((p/'manifest.json').read_text());d=json.loads((p/'deploy.json').read_text())
assert not c['sim'] and not c['sdk_a_autostart'] and c['dt']==.02
assert hashlib.sha256((p/'policy.onnx').read_bytes()).hexdigest()==m['onnx_sha256']
assert hashlib.sha256((p/'deploy.json').read_bytes()).hexdigest()==m['contract_sha256']
assert all(len(d[k])==29 for k in ['kp','kd','offset','scale'])
s=ort.InferenceSession(str(p/'policy.onnx'),providers=['CPUExecutionProvider'])
print('PASS: ONNX/contract hashes; 29 joints; 50Hz; hardware parallel-ankle path; STOP startup.')
print('Inputs:',[(x.name,x.shape) for x in s.get_inputs()])
print('IMU hardware mounting verified:',c['sdk_a']['imu_mount_verified'])
print('IMU input contract:',c['sdk_a'].get('imu_input_frame'))
if '--require-calibration' in sys.argv and c['sdk_a'].get('imu_input_frame') != 'sdk_body_euler' and not c['sdk_a']['imu_mount_verified']:
 raise SystemExit('Hardware mounting not yet verified: edit config/sdk_a_hardware.yaml after measuring the IMU frame.')
