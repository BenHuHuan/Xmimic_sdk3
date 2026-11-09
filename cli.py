"""Unified SDK entry: simulation, onboard, offboard, upload, remote start."""
import argparse,os,sys,subprocess,json,shlex,datetime,ipaddress
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
r=sub.add_parser('run');r.add_argument('--target',choices=['sim','real'],help='Optional: inferred as sim for lo, real otherwise');r.add_argument('--mode',choices=['onboard','offboard'],default='onboard');r.add_argument('--domain',type=int);r.add_argument('--interface');r.add_argument('--robot-ip');r.add_argument('--headless',action='store_true');r.add_argument('--seconds',type=float,default=5);r.add_argument('--dry-run',action='store_true')
for name in ['upload','remote']:
 q=sub.add_parser(name);q.add_argument('--host',required=True,help='SSH alias or user@robot IP');q.add_argument('--path',default='Xrobot3',help='Relative to remote home, or absolute');q.add_argument('--port',type=int,default=22);q.add_argument('--domain',type=int,default=0)
sub.add_parser('check')
a=p.parse_args()
if a.cmd in ('upload','remote'):
 if a.host.startswith('-') or any(x in a.host for x in '\n\r '):p.error('Invalid SSH host')
 if a.cmd=='upload':
  subprocess.run(['ssh','-p',str(a.port),a.host,'mkdir -p -- '+shlex.quote(a.path)],check=True)
  subprocess.run(['rsync','-az','--protect-args','-e',f'ssh -p {a.port}','--exclude=.venv','--exclude=output','--exclude=__pycache__','--exclude=.git',str(ROOT)+'/',a.host+':'+a.path+'/'],check=True)
  print('Uploaded; first installation on robot: cd '+a.path+' && bash install.sh')
 else:
  command='cd '+shlex.quote(a.path)+' && exec ./xrobot3 run --target real --mode onboard --domain '+str(a.domain)
  subprocess.run(['ssh','-t','-p',str(a.port),a.host,command],check=True)
 sys.exit()
if a.cmd=='check':
 subprocess.run([sys.executable,str(ROOT/'sdk/local_setup/check_sdk_a_hardware.py')],check=True)
 import mujoco
 m=mujoco.MjModel.from_xml_path(str(ROOT/'sdk/policy/sdk_a_to/scene_unitree_visual.xml'))
 print('Portable simulator model loaded:',m.nq,m.nv);sys.exit()
if a.target is None:
 if not a.interface:p.error('Specify --interface lo (simulation), a real interface, or --target')
 a.target='sim' if a.interface=='lo' else 'real'
if a.target=='sim' and not a.interface:a.interface='lo'
if a.interface=='lo' and not a.robot_ip:a.robot_ip='127.0.0.1'
if a.target=='real' and not a.robot_ip:a.robot_ip='192.168.41.1'
env=os.environ.copy();env['PYTHONUNBUFFERED']='1';env['ROS_DOMAIN_ID']=str(a.domain if a.domain is not None else (174 if a.target=='sim' else 0))
if a.target=='sim':
 env['ROS_LOCALHOST_ONLY']='1';env['MUJOCO_GL']='glfw'
 env['XMIGCS_CONFIG']=str(ROOT/'sdk/config/sdk_a_sim.yaml')
 out=ROOT/'output'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
 command=[sys.executable,str(ROOT/'sdk/policy/sdk_a_to/run_chain.py'),'pair','--standard-sdk','--output',str(out)]
 command+=['--cases','normal_p0','--seconds',str(a.seconds)] if a.headless else ['--interactive']
else:
 env['ROS_LOCALHOST_ONLY']='0';env['XMIGCS_CONFIG']=str(ROOT/'sdk/config/sdk_a_hardware.yaml')
 command=[sys.executable,str(ROOT/'sdk/rl_control_node.py')]
if a.mode=='offboard' and (a.target=='real' or a.interface or a.robot_ip):
 if not a.interface or not a.robot_ip:p.error('offboard requires --interface and --robot-ip')
 env['ROS_LOCALHOST_ONLY']='0'
 ipaddress.ip_address(a.robot_ip)
 info=json.loads(subprocess.check_output(['ip','-j','-4','addr','show','dev',a.interface]));ips=[x['local'] for dev in info for x in dev['addr_info'] if x['family']=='inet']
 if not ips:p.error('Selected interface has no IPv4 address')
 # Explicit UDP interface and robot discovery peer; remote ROS domain must match.
 xml=f'''<?xml version="1.0"?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors><transport_descriptor><transport_id>wired</transport_id><type>UDPv4</type><interfaceWhiteList><address>{ips[0]}</address></interfaceWhiteList></transport_descriptor></transport_descriptors><participant profile_name="wired_robot" is_default_profile="true"><rtps><userTransports><transport_id>wired</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports><builtin><initialPeersList><locator><udpv4><address>{a.robot_ip}</address></udpv4></locator></initialPeersList></builtin></rtps></participant></profiles>'''
 folder=ROOT/'output';folder.mkdir(exist_ok=True);profile=folder/f'dds_{os.getpid()}.xml';profile.write_text(xml)
 env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
print('Endpoint:',a.target,'interface:',a.interface,'peer:',a.robot_ip,'| FSM: STOP / ZERO / SDK_A',flush=True)
if a.dry_run:print('Command:',shlex.join(command),'domain:',env['ROS_DOMAIN_ID'],flush=True)
if not a.dry_run:
 # Preserve one dedicated console descriptor; ROS/native/Python logs go to disk.
 console_fd=os.dup(sys.stdout.fileno());os.set_inheritable(console_fd,True)
 env['TG_CONSOLE_FD']=str(console_fd)
 logdir=ROOT/'output'/'terminal_logs';logdir.mkdir(parents=True,exist_ok=True)
 logpath=logdir/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}.log')
 print('Runtime log: '+str(logpath),flush=True)
 with logpath.open('a',buffering=1) as logfile:
  os.dup2(logfile.fileno(),1);os.dup2(logfile.fileno(),2)
 os.chdir(ROOT/'sdk');os.execve(command[0],command,env)
