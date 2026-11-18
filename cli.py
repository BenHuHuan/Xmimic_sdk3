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
def _render_bar(label,pct,detail,done=False):
 tty=sys.stdout.isatty()
 fill_ch,empty_ch=('█','░') if 'utf' in (sys.stdout.encoding or '').lower() else ('#','-')
 cols=shutil.get_terminal_size((100,20)).columns or 100
 prefix=f'{label} ['
 suffix=f'] {pct:5.1f}%  {detail}'
 bw=min(28,max(8,cols-len(prefix)-len(suffix)-1)) if tty else 24
 filled=min(bw,max(0,int(round(bw*pct/100.0))))
 line=prefix+fill_ch*filled+empty_ch*(bw-filled)+suffix
 if tty:
  line=line[:max(1,cols-1)]
  sys.stdout.write('\r'+line+'\033[K'+('\n' if done else ''));sys.stdout.flush()
 elif done:
  print(line,flush=True)

def _maybe_notify_sdk_update(repo='BenHuHuan/Xmimic_sdk3',branch='main'):
 """Non-blocking 24h-cached update reminder so users don't miss new SDK features."""
 import urllib.request
 state_file=ROOT/'output'/'.sdk_update_check.json'
 now=time.time()
 try:
  if state_file.exists():
   cached=json.loads(state_file.read_text())
   if now-float(cached.get('checked_at',0))<86400:
    if cached.get('update_available') and cached.get('remote_sha'):
     print(f"[xrobot3] SDK update available on {branch} ({cached['remote_sha'][:7]}). Run './xrobot3 sync' to update.",flush=True)
    return
 except Exception:pass
 update_avail=False;remote_sha=''
 try:
  req=urllib.request.Request(f'https://api.github.com/repos/{repo}/commits/{branch}',headers={'Accept':'application/vnd.github.sha','User-Agent':'xrobot3-cli'})
  with urllib.request.urlopen(req,timeout=0.8) as resp:
   remote_sha=resp.read().decode('utf-8','ignore').strip().strip('"')
  if remote_sha:
   if (ROOT/'.git').exists():
    head=subprocess.run(['git','-C',str(ROOT),'rev-parse','HEAD'],capture_output=True,text=True,timeout=1)
    if head.returncode==0:
     anc=subprocess.run(['git','-C',str(ROOT),'merge-base','--is-ancestor',remote_sha,'HEAD'],capture_output=True,timeout=1)
     update_avail=anc.returncode!=0
   else:
    sync_state=ROOT/'output'/'.sdk_sync_state.json'
    cur_sha=json.loads(sync_state.read_text()).get('sha','') if sync_state.exists() else ''
    update_avail=bool(cur_sha and cur_sha!=remote_sha)
 except Exception:return
 try:
  state_file.parent.mkdir(parents=True,exist_ok=True)
  state_file.write_text(json.dumps({'checked_at':now,'remote_sha':remote_sha,'update_available':update_avail}))
 except Exception:pass
 if update_avail and remote_sha:
  print(f"[xrobot3] SDK update available on {branch} ({remote_sha[:7]}). Run './xrobot3 sync' to update.",flush=True)

def run_sdk_sync(remote='origin',branch='main',repo='BenHuHuan/Xmimic_sdk3',reinstall=False,tarball=False,dry_run=False):
 """Auto-update SDK files from the latest main release with progress bars."""
 import tarfile,tempfile,urllib.request
 use_git=(ROOT/'.git').exists() and not tarball
 if dry_run:
  if use_git:
   print('Plan (git):',shlex.join(['git','-C',str(ROOT),'fetch',remote,branch]),'->',shlex.join(['git','-C',str(ROOT),'merge','--no-edit',f'{remote}/{branch}']))
  else:
   print(f'Plan (archive): download https://github.com/{repo}/archive/refs/heads/{branch}.tar.gz -> sync files into {ROOT}')
  return
 deps_before={f:(ROOT/f).read_bytes() for f in ('requirements.txt','install.sh') if (ROOT/f).exists()}
 if use_git:
  def _git(*args,check=True):
   return subprocess.run(['git','-C',str(ROOT),*args],check=check,text=True,capture_output=True)
  _render_bar('Checking SDK',15.0,f'fetching {remote}/{branch}...')
  cur=_git('branch','--show-current').stdout.strip() or 'HEAD'
  ref=f'{remote}/{branch}'
  _git('fetch',remote,branch)
  _render_bar('Checking SDK',60.0,f'comparing {cur} vs {ref}...')
  if cur!=branch and _git('show-ref','--verify',f'refs/heads/{branch}',check=False).returncode==0:
   if _git('merge-base','--is-ancestor',branch,ref,check=False).returncode==0:
    _git('branch','-f',branch,ref)
  behind=int(_git('rev-list','--count',f'HEAD..{ref}').stdout.strip() or '0')
  short_sha=_git('rev-parse','--short',ref).stdout.strip()
  if behind==0:
   _render_bar('SDK Sync',100.0,f'already up to date ({branch} @ {short_sha})',done=True)
  else:
   changed=[x for x in _git('diff','--name-only',f'HEAD..{ref}').stdout.splitlines() if x.strip()]
   dirty=bool(_git('status','--porcelain','--untracked-files=no').stdout.strip())
   stashed=False
   if dirty:
    _render_bar('Updating SDK',20.0,'stashing local changes...')
    _git('stash','push','-m','xrobot3-sync-autostash')
    stashed=True
   try:
    _render_bar('Updating SDK',45.0,f'applying {behind} commit(s) ({len(changed)} files)...')
    _git('merge','--ff-only',ref) if cur==branch else _git('merge','--no-edit',ref)
   except subprocess.CalledProcessError:
    print(f'\nSync hit a merge conflict with {ref}. Resolve conflicts manually.',file=sys.stderr)
    if stashed:print('Local edits saved in git stash ("xrobot3-sync-autostash"); run `git stash pop` after resolving.',file=sys.stderr)
    sys.exit(1)
   if stashed and _git('stash','pop',check=False).returncode!=0:
    print('\nSDK updated, but `git stash pop` had conflicts. Resolve manually.',file=sys.stderr)
    sys.exit(1)
   total=max(1,len(changed))
   for i,rel in enumerate(changed,1):
    if i==total or i%5==0:
     _render_bar('Updating SDK',45.0+55.0*(i/total),f'{i}/{total} files ({rel[:28]})',done=(i==total))
   if not changed:
    _render_bar('Updating SDK',100.0,f'synced {behind} commit(s) @ {short_sha}',done=True)
   print(f'SDK updated: {cur} synced with {ref} ({behind} new commit(s), @ {short_sha}).')
 else:
  _render_bar('Checking SDK',10.0,f'querying {repo}@{branch}...')
  remote_sha=''
  try:
   req=urllib.request.Request(f'https://api.github.com/repos/{repo}/commits/{branch}',headers={'Accept':'application/vnd.github.sha','User-Agent':'xrobot3-cli'})
   with urllib.request.urlopen(req,timeout=5) as resp:
    remote_sha=resp.read().decode('utf-8','ignore').strip().strip('"')
  except Exception:pass
  sync_state=ROOT/'output'/'.sdk_sync_state.json'
  if remote_sha and not reinstall and sync_state.exists():
   try:
    if json.loads(sync_state.read_text()).get('sha')==remote_sha:
     _render_bar('SDK Sync',100.0,f'already up to date ({branch} @ {remote_sha[:7]})',done=True)
     return
   except Exception:pass
  url=f'https://github.com/{repo}/archive/refs/heads/{branch}.tar.gz'
  req=urllib.request.Request(url,headers={'User-Agent':'xrobot3-cli'})
  t0=time.monotonic()
  with tempfile.TemporaryDirectory(prefix='xrobot3_sync_') as tmp:
   tgz=Path(tmp)/'sdk.tar.gz'
   with urllib.request.urlopen(req,timeout=30) as resp,tgz.open('wb') as out_f:
    total_b=int(resp.headers.get('Content-Length') or 0)
    done_b=0
    while True:
     chunk=resp.read(65536)
     if not chunk:break
     out_f.write(chunk);done_b+=len(chunk)
     dt=max(0.05,time.monotonic()-t0);rate=done_b/dt
     pct=(min(100.0,done_b*100.0/total_b) if total_b>0 else min(95.0,done_b/2e6*10.0))
     detail=(f'{_fmt_bytes(done_b)}/{_fmt_bytes(total_b)}  {_fmt_bytes(rate)}/s' if total_b>0 else f'{_fmt_bytes(done_b)}  {_fmt_bytes(rate)}/s')
     _render_bar('Downloading',pct,detail)
   _render_bar('Downloading',100.0,f'{_fmt_bytes(done_b)}  {_fmt_time(time.monotonic()-t0)}',done=True)
   with tarfile.open(tgz,'r:gz') as tf:
    tf.extractall(tmp)
   roots=[p for p in Path(tmp).iterdir() if p.is_dir()]
   if not roots:raise RuntimeError('Empty SDK archive')
   src_root=roots[0]
   skip={'.git','.venv','.venv-mj4','output','vendor','.vscode','.idea','.claude','__pycache__'}
   files=[f for f in src_root.rglob('*') if f.is_file() and not any(part in skip for part in f.relative_to(src_root).parts)]
   total_f=max(1,len(files))
   for i,src in enumerate(files,1):
    rel=src.relative_to(src_root)
    dst=ROOT/rel
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    if i==total_f or i%10==0:
     _render_bar('Updating SDK',i*100.0/total_f,f'{i}/{total_f} files',done=(i==total_f))
  sync_state.parent.mkdir(parents=True,exist_ok=True)
  sync_state.write_text(json.dumps({'sha':remote_sha,'branch':branch,'updated_at':time.time()}))
  print(f'SDK updated from {repo}@{branch}'+(f' ({remote_sha[:7]})' if remote_sha else '')+'.')
 try:
  chk=ROOT/'output'/'.sdk_update_check.json'
  if chk.exists():chk.unlink()
 except Exception:pass
 deps_after={f:(ROOT/f).read_bytes() for f in ('requirements.txt','install.sh') if (ROOT/f).exists()}
 if reinstall or deps_before!=deps_after:
  print('Dependencies changed; refreshing environment via install.sh...')
  subprocess.run(['bash',str(ROOT/'install.sh')],cwd=str(ROOT),check=True)

def gpu_render_check():
 """Warn when the NVIDIA GPU is power/thermal-capped; the viewer then drops to a few FPS."""
 try:
  q=subprocess.run(['nvidia-smi','--query-gpu=name,enforced.power.limit,power.default_limit,clocks_event_reasons.sw_thermal_slowdown','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3).stdout.strip().splitlines()[0]
  name,limit,default,thermal=[x.strip() for x in q.split(',')]
  if float(limit)<0.5*float(default) or thermal=='Active':
   print(f'WARNING: {name} capped at {float(limit):.0f} W (default {float(default):.0f} W), SW thermal slowdown={thermal}. '
         'The MuJoCo viewer will stutter. Fix: select the laptop Performance mode on AC, and install nvidia-powerd: '
         'sudo cp /usr/share/doc/nvidia-kernel-common-*/nvidia-powerd.service /etc/systemd/system/ && '
         'sudo systemctl daemon-reload && sudo systemctl enable --now nvidia-powerd',flush=True)
 except Exception:pass
p=argparse.ArgumentParser(prog='xrobot3',description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
r=sub.add_parser('run',help='Run local simulation, wired offboard, onboard, or SSH remote control');r.add_argument('--target',choices=['sim','real'],help=argparse.SUPPRESS);r.add_argument('--mode',choices=['onboard','offboard'],default='onboard',help=argparse.SUPPRESS);r.add_argument('--domain',type=int);r.add_argument('--interface',help='Network adapter: lo starts simulation; any other adapter connects to the robot');r.add_argument('--robot-ip');r.add_argument('--headless',action='store_true');r.add_argument('--seconds',type=float,default=5);r.add_argument('--dry-run',action='store_true')
r.add_argument('--physics',choices=['4.0','3.3.7','dev','stable','raisim'],default='4.0',help='Simulator backend/renderer: 4.0/dev=MuJoCo 4.0 (.venv-mj4, default), 3.3.7/stable=MuJoCo 3.3.7 (.venv), raisim=RaiSim + Rayrai renderer')
r.add_argument('--raisim-renderer',choices=['rayrai','tcp'],default='rayrai',help='RaiSim visualizer when --physics raisim: rayrai (default) or tcp (RaisimServer)')
r.add_argument('--host',help='SSH alias or user@robot IP: run the SDK on the onboard computer')
r.add_argument('--path',default='Xrobot3',help='Project path on the onboard computer')
r.add_argument('--port',type=int,default=22,help='SSH port')
_SUB_HELP={'upload':'Sync workspace to remote robot over SSH','deploy':'Sync workspace and run install.sh on remote robot','remote':'Launch SDK on remote robot over SSH','body':'Start or stop remote onboard body_control driver'}
for name in ['upload','deploy','remote','body']:
 q=sub.add_parser(name,help=_SUB_HELP[name]);q.add_argument('--host',required=True,help='SSH alias or user@robot IP');q.add_argument('--path',default='Xrobot3',help='Relative to remote home, or absolute');q.add_argument('--port',type=int,default=22);q.add_argument('--domain',type=int,default=0)
 q.add_argument('--dry-run',action='store_true')
 if name=='body':
  q.add_argument('--stop',action='store_true',help='Stop running body_control process')
sub.add_parser('check',help='Verify Python environment, ROS 2 messages, and ONNX/deploy contracts')
sub.add_parser('interfaces',help='List local IPv4 interfaces for wired/wireless DDS')
sy=sub.add_parser('sync',help='Auto-update SDK files to the latest main release with progress bar')
sy.add_argument('--remote',default='origin',help='Git remote name when in a git checkout (default: origin)')
sy.add_argument('--branch',default='main',help='Source branch to update from (default: main)')
sy.add_argument('--repo',default='BenHuHuan/Xmimic_sdk3',help='GitHub owner/repo for archive updates (default: BenHuHuan/Xmimic_sdk3)')
sy.add_argument('--reinstall',action='store_true',help='Re-run install.sh after updating SDK files')
sy.add_argument('--tarball',action='store_true',help='Force downloading release archive instead of using git')
sy.add_argument('--dry-run',action='store_true',help='Print update plan without modifying files')
a=p.parse_args()
if a.cmd=='sync':
 run_sdk_sync(remote=a.remote,branch=a.branch,repo=a.repo,reinstall=a.reinstall,tarball=a.tarball,dry_run=a.dry_run)
 sys.exit()
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
 _maybe_notify_sdk_update()
 subprocess.run([sys.executable,str(ROOT/'sdk/local_setup/check_sdk_a_hardware.py')],check=True)
 mj4_py=ROOT/'.venv-mj4/bin/python'
 sim_py=str(mj4_py) if os.access(mj4_py,os.X_OK) else sys.executable
 subprocess.run([sim_py,'-c',f"import mujoco; m=mujoco.MjModel.from_xml_path({str(ROOT/'sdk/policy/sdk_a_to/scene_unitree_visual.xml')!r}); print('Portable simulator model loaded:', m.nq, m.nv, '(MuJoCo', mujoco.__version__ + ')')"],check=True)
 subprocess.run([sim_py,'-c',f"import json, sys, tempfile; from pathlib import Path; sys.path.insert(0, {str(ROOT/'sdk/policy/sdk_a_to')!r}); from raisim_urdf import mjcf_to_raisim_urdf; d=tempfile.mkdtemp(); m=mjcf_to_raisim_urdf({str(ROOT/'sdk/policy/sdk_a_to/scene_unitree_visual.xml')!r}, Path(d)/'tiangong3.urdf'); c=json.loads(Path({str(ROOT/'sdk/policy/sdk_a_to/deploy.json')!r}).read_text()); assert m['joint_names']==c['joint_names']; print('RaiSim URDF contract verified:', len(m['joint_names']), 'joints, mass=', round(m['total_mass'], 2), 'kg')"],check=True)
 sys.exit()
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
 python=sys.executable
 use_raisim=a.physics=='raisim'
 use_mj4=a.physics in ('4.0','dev')
 if use_mj4:
  # Default: MuJoCo 4.0 core bindings rebuilt for the ROS Python (vendor/wheels); same SDK/DDS path.
  python=str(ROOT/'.venv-mj4/bin/python')
  if not os.access(python,os.X_OK):p.error(f'--physics {a.physics} needs .venv-mj4; run: bash scripts/setup_mujoco4_dev.sh (or pass --physics 3.3.7)')
  env['VIRTUAL_ENV']=str(ROOT/'.venv-mj4');env['PATH']=str(ROOT/'.venv-mj4/bin')+os.pathsep+env.get('PATH','')
 if use_raisim:
  print('Physics:',a.physics,f'(RaiSim + {a.raisim_renderer})',flush=True)
 else:
  print('Physics:',a.physics,'(MuJoCo 4.0)' if use_mj4 else '(MuJoCo 3.3.7)',flush=True)
 if not a.headless:gpu_render_check()
 command=[python,str(ROOT/'sdk/policy/sdk_a_to/run_chain.py'),'pair','--standard-sdk','--output',str(out)]
 if use_raisim:
  command+=['--physics','raisim','--raisim-renderer',a.raisim_renderer]
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
 _maybe_notify_sdk_update()
 # Preserve one dedicated console descriptor; ROS/native/Python logs go to disk.
 console_fd=os.dup(sys.stdout.fileno());os.set_inheritable(console_fd,True)
 env['TG_CONSOLE_FD']=str(console_fd)
 logdir=ROOT/'output'/'terminal_logs';logdir.mkdir(parents=True,exist_ok=True)
 logpath=logdir/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}.log')
 print('Runtime log: '+str(logpath),flush=True)
 with logpath.open('a',buffering=1) as logfile:
  os.dup2(logfile.fileno(),1);os.dup2(logfile.fileno(),2)
 os.chdir(ROOT/'sdk');os.execve(command[0],command,env)
