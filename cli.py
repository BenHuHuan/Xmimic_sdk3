"""Unified SDK entry: simulation, onboard, offboard, upload, remote start."""
import argparse,os,re,sys,shutil,subprocess,json,shlex,datetime,ipaddress,time
from pathlib import Path
from network import interfaces,interface_ipv4,dds_profile
ROOT=Path(__file__).resolve().parent

def _fmt_bytes(n):
 n=float(n)
 for u in ('B','KB','MB','GB','TB'):
  if n<1024 or u=='TB':return f'{int(n)} B' if u=='B' else f'{n:.1f} {u}'
  n/=1024

def _fmt_time(sec):
 sec=max(0,int(sec));h,rem=divmod(sec,3600);m,s=divmod(rem,60)
 return f'{h}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}'

def _parse_speed(text):
 m=re.match(r'^([\d.]+)([kKMGT]?B)/s$',text)
 if not m:return 0.0
 return float(m.group(1))*{'B':1,'kB':1024,'KB':1024,'MB':1024**2,'GB':1024**3,'TB':1024**4}[m.group(2)]

def run_rsync_with_progress(rsync_base):
 env={**os.environ,'LC_ALL':'C'}
 tty=sys.stdout.isatty()
 fill_ch,empty_ch=('█','░') if 'utf' in (sys.stdout.encoding or '').lower() else ('#','-')
 if tty:sys.stdout.write('\rScanning files...\033[K');sys.stdout.flush()
 probe=subprocess.run([rsync_base[0],'-n','--stats','--no-h',*rsync_base[1:]],capture_output=True,text=True,env=env)
 total_bytes=total_files=None
 if probe.returncode==0:
  mb=re.search(r'Total transferred file size:\s*(\d+)',probe.stdout)
  mf=re.search(r'Number of regular files transferred:\s*(\d+)',probe.stdout)
  if mb:total_bytes=int(mb.group(1))
  if mf:total_files=int(mf.group(1))
 cmd=[rsync_base[0],'--no-inc-recursive','--no-h','--info=progress2',*rsync_base[1:]]
 def render(pct,done_b,speed_s,eta_s,files_done,done=False):
  cols=shutil.get_terminal_size((100,20)).columns or 100
  size_part=f'{_fmt_bytes(done_b)}/{_fmt_bytes(total_bytes)}' if total_bytes is not None else _fmt_bytes(done_b)
  file_part=f' ({files_done}/{total_files})' if total_files is not None else (f' ({files_done} files)' if files_done else '')
  tail=eta_s if done else f'ETA {eta_s}'
  prefix='Uploading ['
  suffix=f'] {pct:5.1f}%  {size_part}  {speed_s}  {tail}{file_part}'
  bw=min(28,max(8,cols-len(prefix)-len(suffix)-1)) if tty else 24
  filled=min(bw,max(0,int(round(bw*pct/100.0))))
  line=prefix+fill_ch*filled+empty_ch*(bw-filled)+suffix
  return line[:max(1,cols-1)] if tty and len(line)>=cols else line
 t0=time.monotonic();done_bytes=0;files_done=0;last_speed='0 B/s';last_non_tty_pct=-10.0;wrote_tty=tty
 proc=subprocess.Popen(cmd,stdout=subprocess.PIPE,env=env)
 try:
  if not tty and total_bytes is not None:
   print(f'Uploading {_fmt_bytes(total_bytes)}'+(f' ({total_files} files)...' if total_files is not None else '...'),flush=True)
  buf=bytearray()
  while True:
   ch=proc.stdout.read(1)
   if not ch:break
   if ch in (b'\r',b'\n'):
    if buf:
     line=buf.decode('utf-8','replace');buf.clear()
     m=re.match(r'\s*(\d+)\s+(\d+)%\s+(\S+/s)',line)
     if m:
      done_bytes=int(m.group(1));rsync_pct=int(m.group(2));last_speed=m.group(3)
      xm=re.search(r'xfr#(\d+)',line)
      if xm:files_done=int(xm.group(1))
      pct=(100.0 if total_bytes==0 else min(100.0,done_bytes*100.0/total_bytes)) if total_bytes is not None else float(rsync_pct)
      rate=_parse_speed(last_speed);elapsed=time.monotonic()-t0
      if rate<=0 and elapsed>0.2 and done_bytes>0:rate=done_bytes/elapsed
      eta=_fmt_time((total_bytes-done_bytes)/rate) if (total_bytes and rate>0 and done_bytes<total_bytes) else ('00:00' if total_bytes is not None and done_bytes>=total_bytes else '--:--')
      if tty:
       sys.stdout.write('\r'+render(pct,done_bytes,last_speed,eta,files_done)+'\033[K');sys.stdout.flush();wrote_tty=True
      elif pct>=last_non_tty_pct+10.0:
       print(render(pct,done_bytes,last_speed,eta,files_done),flush=True);last_non_tty_pct=pct
   else:buf.extend(ch)
  rc=proc.wait()
 except BaseException:
  if proc.poll() is None:proc.terminate();proc.wait()
  if wrote_tty:sys.stdout.write('\n');sys.stdout.flush()
  raise
 if rc!=0:
  if wrote_tty:sys.stdout.write('\n');sys.stdout.flush()
  raise subprocess.CalledProcessError(rc,cmd)
 elapsed=time.monotonic()-t0
 final_bytes=total_bytes if total_bytes is not None else done_bytes
 final_files=total_files if total_files is not None else files_done
 if total_bytes==0 and (total_files or 0)==0:
  msg=render(100.0,0,'up to date',_fmt_time(elapsed),0,done=True)
 else:
  avg=_fmt_bytes(final_bytes/max(elapsed,1e-3))+'/s'
  msg=render(100.0,final_bytes,avg,_fmt_time(elapsed),final_files,done=True)
 if tty:sys.stdout.write('\r'+msg+'\033[K\n');sys.stdout.flush()
 else:print(msg,flush=True)
p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
r=sub.add_parser('run');r.add_argument('--target',choices=['sim','real'],help=argparse.SUPPRESS);r.add_argument('--mode',choices=['onboard','offboard'],default='onboard',help=argparse.SUPPRESS);r.add_argument('--domain',type=int);r.add_argument('--interface',help='Network adapter: lo starts simulation; any other adapter connects to the robot');r.add_argument('--robot-ip');r.add_argument('--headless',action='store_true');r.add_argument('--seconds',type=float,default=5);r.add_argument('--dry-run',action='store_true')
r.add_argument('--host',help='SSH alias or user@robot IP: run the SDK on the onboard computer')
r.add_argument('--path',default='Xrobot3',help='Project path on the onboard computer')
r.add_argument('--port',type=int,default=22,help='SSH port')
for name in ['upload','deploy','remote','body']:
 q=sub.add_parser(name);q.add_argument('--host',required=True,help='SSH alias or user@robot IP');q.add_argument('--path',default='Xrobot3',help='Relative to remote home, or absolute');q.add_argument('--port',type=int,default=22);q.add_argument('--domain',type=int,default=0)
 q.add_argument('--dry-run',action='store_true')
 if name=='body':
  q.add_argument('--stop',action='store_true',help='Stop running body_control process')
sub.add_parser('check')
sub.add_parser('interfaces',help='List local IPv4 interfaces for wired/wireless DDS')
a=p.parse_args()
if a.cmd=='interfaces':
 for item in interfaces():print(item['name'],item['kind'],', '.join(item['addresses']) or '(no IPv4)')
 sys.exit()
if a.cmd in ('upload','deploy','remote','body') or (a.cmd=='run' and a.host):
 if a.host.startswith('-') or any(x in a.host for x in '\n\r '):p.error('Invalid SSH host')
 ssh_opts=['-p',str(a.port),'-o','ControlMaster=auto','-o','ControlPath=/tmp/xrobot3-ssh-%C','-o','ControlPersist=60s']
 if a.cmd in ('upload','deploy'):
  excludes=['--exclude=output','--exclude=__pycache__','--exclude=*.pyc','--exclude=.git','--exclude=*.log','--exclude=.venv','--exclude=.vscode','--exclude=.idea','--exclude=.pytest_cache','--exclude=.mypy_cache','--exclude=.ruff_cache']
  if a.cmd=='upload':excludes=['--exclude=vendor']+excludes
  rsync_base=['rsync','-az','--protect-args','-e',shlex.join(['ssh',*ssh_opts]),*excludes,str(ROOT)+'/',a.host+':'+a.path+'/']
  commands=[['ssh',*ssh_opts,a.host,'mkdir -p -- '+shlex.quote(a.path)],rsync_base]
  if a.cmd=='deploy':commands.append(['ssh','-t',*ssh_opts,a.host,'cd '+shlex.quote(a.path)+' && bash install.sh'])
  for command in commands:
   if a.dry_run:
    shown=[command[0],'--no-inc-recursive','--info=progress2',*command[1:]] if command[0]=='rsync' else command
    print('Command:',shlex.join(shown))
   elif command[0]=='rsync':run_rsync_with_progress(command)
   else:subprocess.run(command,check=True)
  if not a.dry_run:print('Deployment complete.' if a.cmd=='deploy' else 'Uploaded. Use deploy to install the environment too.')
 elif a.cmd=='body':
  domain=str(a.domain if a.domain is not None else 0)
  if a.stop:
   command='sudo pkill -INT -f body_control.launch.py'
  else:
   inner=f'source /home/ubuntu/xos/setup.bash && export ROS_DOMAIN_ID={shlex.quote(domain)} && exec chrt -r 99 taskset -c 6-9 ros2 launch body_control body_control.launch.py'
   command='sudo bash -c '+shlex.quote(inner)
  ssh_command=['ssh','-t',*ssh_opts,a.host,command]
  if a.dry_run:print('Command:',shlex.join(ssh_command))
  else:subprocess.run(ssh_command,check=True)
 else:
  domain=str(a.domain if a.domain is not None else 0)
  remote_args=['./xrobot3','run','--target','real','--mode','onboard','--domain',domain]
  if a.cmd=='run':
   if a.target=='sim' or a.interface=='lo' or a.headless:p.error('--host starts onboard hardware control; simulation options are not supported')
   if a.interface:remote_args+=['--interface',a.interface]
   if a.robot_ip:remote_args+=['--robot-ip',a.robot_ip]
  command='cd '+shlex.quote(a.path)+' && exec '+shlex.join(remote_args)
  ssh_command=['ssh','-t',*ssh_opts,a.host,command]
  if getattr(a,'dry_run',False):print('Command:',shlex.join(ssh_command))
  else:subprocess.run(ssh_command,check=True)
 sys.exit()
if a.cmd=='check':
 subprocess.run([sys.executable,str(ROOT/'sdk/local_setup/check_sdk_a_hardware.py')],check=True)
 import mujoco
 m=mujoco.MjModel.from_xml_path(str(ROOT/'sdk/policy/sdk_a_to/scene_unitree_visual.xml'))
 print('Portable simulator model loaded:',m.nq,m.nv);sys.exit()
if a.interface:
 inferred_target='sim' if a.interface=='lo' else 'real'
 if a.target is not None and a.target!=inferred_target:p.error('--target conflicts with --interface; omit --target')
 a.target=inferred_target
elif a.target is None:
 p.error('Specify --interface lo for simulation, or --interface with your robot-facing adapter; list adapters with ./xrobot3 interfaces')
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
# An explicit interface selects the transport regardless of launcher mode.
# Both wired and wireless adapters use the same SDK and DDS command path.
if a.mode=='offboard' and a.target=='real' and not a.interface:
 p.error('offboard requires --interface; run ./xrobot3 interfaces')
if a.interface:
 if not a.robot_ip:a.robot_ip='127.0.0.1' if a.target=='sim' else '192.168.41.1'
 try:profile_text=dds_profile(interface_ipv4(a.interface),a.robot_ip)
 except ValueError as exc:p.error(str(exc))
 folder=ROOT/'output';folder.mkdir(exist_ok=True)
 profile=folder/f'dds_{os.getpid()}.xml';profile.write_text(profile_text)
 # Explicit interface/peer replaces inherited transport selection.
 env.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
 env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
 env['ROS_LOCALHOST_ONLY']='1' if a.target=='sim' and a.interface=='lo' else '0'
print('Endpoint:',a.target,'interface:',a.interface,'peer:',a.robot_ip,'| FSM: STOP / ZERO / FIX / SDK-A / WALK',flush=True)
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
