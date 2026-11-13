"""Separate GPU viewer; physics never waits for window sync or OpenGL locks."""
from pathlib import Path
import subprocess,sys,time,json,socket
import mujoco
import numpy as np
from glfw import (
    KEY_F9, KEY_F10, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
    KEY_PAGE_UP, KEY_PAGE_DOWN, KEY_EQUAL, KEY_MINUS,
    KEY_KP_0, KEY_KP_1, KEY_KP_2, KEY_KP_3, KEY_KP_4,
    KEY_KP_5, KEY_KP_6, KEY_KP_7, KEY_KP_8, KEY_KP_9,
    KEY_KP_ADD, KEY_KP_SUBTRACT,
)

HANG_KEYS={
    KEY_F9:'ON', KEY_F10:'OFF', KEY_KP_0:'TOGGLE', KEY_KP_5:'TOGGLE',
    ord('7'):'UP', KEY_KP_7:'UP', KEY_KP_9:'UP', KEY_PAGE_UP:'UP',
    ord('8'):'DOWN', KEY_KP_1:'DOWN', KEY_KP_3:'DOWN', KEY_PAGE_DOWN:'DOWN',
    KEY_UP:'STRONGER', KEY_RIGHT:'STRONGER', KEY_KP_8:'STRONGER', KEY_KP_6:'STRONGER', KEY_KP_ADD:'STRONGER', KEY_EQUAL:'STRONGER',
    KEY_DOWN:'WEAKER', KEY_LEFT:'WEAKER', KEY_KP_2:'WEAKER', KEY_KP_4:'WEAKER', KEY_KP_SUBTRACT:'WEAKER', KEY_MINUS:'WEAKER',
}

class RenderBridge:
    def __init__(self,model,output):
        self.path=Path(output)/'viewer_shared.npy'
        self.key_socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.key_path=str(Path(output)/'viewer_keys.sock')
        Path(self.key_path).unlink(missing_ok=True)
        self.key_socket.bind(self.key_path);self.key_socket.setblocking(False)
        self.nq=model.nq;self.nv=model.nv;self.nb=model.nbody;self.nu=model.nu
        self.state_len=self.nq+self.nv
        self.frc_offset=32+self.state_len
        self.ovr_offset=self.frc_offset+self.nb*6
        self.jitter_offset=self.ovr_offset+self.state_len
        self.shared=np.lib.format.open_memmap(self.path,mode='w+',dtype=np.float64,shape=(self.jitter_offset+self.nu,))
        self.shared[:]=0
        self.shared[0]=1
        self.shared[10]=float(model.opt.timestep)
        self.shared[13]=1.0  # sim.run (1=Run, 0=Pause)
        self.shared[14]=1.0  # real-time slowdown factor
        self._last_ovr_seq=0
        self._ctrl_noise=np.zeros(model.nu)
        self.proc=subprocess.Popen([sys.executable,__file__,str(self.path),str(self.nq),str(self.nv),str(self.nb),str(self.nu)])
    def update(self,data,performance=None,jitter_monitor=None):
        a=self.shared;a[1]+=1
        a[32:32+self.nq]=data.qpos;a[32+self.nq:32+self.state_len]=data.qvel
        now=time.monotonic()
        if not hasattr(self,'last_clock'):self.last_clock=(now,data.time)
        elapsed=now-self.last_clock[0]
        if elapsed>=0.5:
            a[6]=(data.time-self.last_clock[1])/elapsed
            self.last_clock=(now,data.time)
        perf=performance or {}
        fresh=time.time()-perf.get('wall_time',0)<2
        a[7]=perf.get('control_hz',0) if fresh else 0
        a[8]=perf.get('fsm_ms',0) if fresh else 0
        a[9]={'STOP':0,'ZERO':1,'SDK_A':2,'DEXAMP':3,'FIX':4}.get(perf.get('state'),-1) if fresh else -1
        a[22]=float(perf.get('filter_alpha',0.0)) if fresh else 0.0
        if jitter_monitor is not None:
            a[23]=jitter_monitor.jitter_score
            a[24]=jitter_monitor.twitch_score
            a[25]=jitter_monitor.peak_freq_hz
            a[26]=float(jitter_monitor.peak_joint_idx)
            a[27]=float(jitter_monitor.active_joints)
            a[self.jitter_offset:self.jitter_offset+self.nu]=jitter_monitor.severity
        a[2]=data.time;a[5]=now;a[1]+=1
    def apply_force(self,data):
        if time.monotonic()-self.shared[3]<.2:
            data.xfrc_applied[:]=self.shared[self.frc_offset:self.ovr_offset].reshape(self.nb,6)
    def apply_ctrl_noise(self,model,data):
        std=float(self.shared[16]);rate_s=float(self.shared[17])
        if std>0.0:
            rate=float(np.exp(-model.opt.timestep/max(rate_s,1e-9)))
            scale=std*float(np.sqrt(max(0.0,1.0-rate*rate)))
            self._ctrl_noise=rate*self._ctrl_noise+scale*np.random.standard_normal(model.nu)
            data.ctrl[:]=np.clip(data.ctrl+self._ctrl_noise,-model.actuator_ctrlrange[:,1],model.actuator_ctrlrange[:,1])
    def is_paused(self):
        return self.shared[13]==0.0
    def slowdown(self):
        return max(0.1,float(self.shared[14] or 1.0))
    def poll_state_override(self,model,data,default_qpos,default_qvel):
        seq=int(self.shared[18])
        if seq==self._last_ovr_seq:return False
        self._last_ovr_seq=seq
        kind=int(self.shared[20])
        if kind==1:  # Reset / Reload -> restore initial standing fixture
            data.qpos[:]=default_qpos;data.qvel[:]=default_qvel;data.time=0.0
        else:  # Load key / History scrubber / Paused slider edit
            ovr=self.shared[self.ovr_offset:self.ovr_offset+self.state_len]
            data.qpos[:]=ovr[:self.nq];data.qvel[:]=ovr[self.nq:]
            data.time=max(0.0,float(self.shared[19]))
        data.xfrc_applied[:]=0
        mujoco.mj_forward(model,data)
        self.last_clock=(time.monotonic(),data.time)
        self.update(data)
        self.shared[21]=float(seq)
        return True
    def read_commands(self):
        while True:
            try:yield self.key_socket.recv(64).decode('ascii')
            except BlockingIOError:return
    def is_running(self):return self.proc.poll() is None and bool(self.shared[0])
    def close(self):
        self.shared[0]=0
        self.key_socket.close();Path(self.key_path).unlink(missing_ok=True)
        try:self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:self.proc.terminate();self.proc.wait(timeout=3)

def main():
    import atexit,threading,glfw,mujoco.viewer
    from mujoco import _simulate
    path=Path(sys.argv[1]);nq,nv,nb,nu=map(int,sys.argv[2:]);a=np.load(path,mmap_mode='r+')
    state_len=nq+nv;frc_offset=32+state_len;ovr_offset=frc_offset+nb*6;jitter_offset=ovr_offset+state_len
    xml_path=str(Path(__file__).parent/'scene_unitree_visual.xml')
    model=mujoco.MjModel.from_xml_path(xml_path);data=mujoco.MjData(model)
    model.vis.quality.offsamples=0
    base_geom_rgba=model.geom_rgba.copy()
    joint_names=[];joint_geom_ids=[]
    for j in range(nu):
        jname=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j+1) or f'joint_{j}'
        bid=int(model.jnt_bodyid[j+1])
        geoms=[g for g in range(model.ngeom) if int(model.geom_bodyid[g])==bid]
        joint_names.append(jname.replace('_joint',''))
        joint_geom_ids.append(geoms)

    # Real-time scrolling oscilloscope figure for HF jitter & twitch spikes
    fig=mujoco.MjvFigure()
    mujoco.mjv_defaultFigure(fig)
    fig.title='Real-Time Jitter & Twitch (%)'
    fig.xlabel='History (0.1s/pt)'
    fig.linename[0]='HF Jitter'
    fig.linename[1]='Twitch'
    fig.linergb[0]=[1.0,0.78,0.10]
    fig.linergb[1]=[1.0,0.18,0.15]
    fig.linewidth=2.0
    fig.figurergba=[0.06,0.06,0.08,0.72]
    fig.panergba=[0.02,0.02,0.04,0.65]
    fig.range[0]=[0.0,150.0]
    fig.range[1]=[0.0,110.0]
    fig_n=150
    hist_jitter=np.zeros(fig_n,dtype=np.float32)
    hist_twitch=np.zeros(fig_n,dtype=np.float32)
    default_qpos=model.key_qpos[0].copy() if model.nkey>0 else model.qpos0.copy()
    default_qvel=model.key_qvel[0].copy() if model.nkey>0 else np.zeros(nv)
    while a[5]==0 and a[0]:time.sleep(.01)
    keys=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    rt_state={'idx':0,'lock_until':0.0}
    def key_callback(key):
        if key in (KEY_MINUS,KEY_EQUAL):
            rt_state['lock_until']=time.monotonic()+0.1
        command=HANG_KEYS.get(key)
        if command:
            try:keys.sendto(command.encode('ascii'),str(path.parent/'viewer_keys.sock'))
            except OSError:pass
    cam=mujoco.MjvCamera();opt=mujoco.MjvOption();pert=mujoco.MjvPerturb()
    # Pass run_physics_thread=True so C++ Simulate enables Pause/Run, Reload, Noise, and History UI controls.
    sim=_simulate.Simulate(cam,opt,pert,None,True,key_callback)
    sim.ui0_enable=True;sim.ui1_enable=True
    if not glfw.init():raise RuntimeError('could not initialize GLFW')
    atexit.register(glfw.terminate)

    def bridge_loop():
        reload=True;reload_count=0
        last_qpos=None;last_qvel=None;last_time=0.0
        rendered_frames=0;last_text_time=0.0;last_sync=time.monotonic()
        frames=0;began=last_sync
        while not sim.exitrequest and a[0]:
            if sim.droploadrequest:
                sim.droploadrequest=0
            if sim.uiloadrequest:
                sim.uiloadrequest_decrement()
                reload=True
            if reload:
                data.qpos[:]=default_qpos;data.qvel[:]=default_qvel;data.time=0.0
                mujoco.mj_forward(model,data)
                sim.load_message(xml_path)
                sim.load(model,data,xml_path)
                sim.load_error=''
                with sim.lock():
                    cam.distance=3.5;cam.elevation=-25;cam.lookat[:]=default_qpos[:3]
                    last_qpos=data.qpos.copy();last_qvel=data.qvel.copy();last_time=0.0
                    data.xfrc_applied[0,0]=1e-300
                if reload_count>0:
                    a[20]=1;a[19]=0.0;a[18]+=1
                reload=False;reload_count+=1
                rendered_frames=0;last_sync=time.monotonic()

            # Wait lock-free until RenderLoop finishes Simulate::Sync (zeros xfrc_applied[0,0])
            # so RenderLoop never contends for sim.lock() at VSync wakeup.
            now=time.monotonic()
            if data.xfrc_applied[0,0]!=0.0 and now-last_sync<0.05:
                time.sleep(0.001)
                continue
            last_sync=now

            seq=a[1]
            has_snap=False
            if int(seq)%2==0:
                snapshot=a[32:32+state_len].copy();simtime=float(a[2])
                if a[1]==seq:has_snap=True

            with sim.lock():
                if data.xfrc_applied[0,0]==0.0:
                    rendered_frames+=1;frames+=1
                if now<rt_state['lock_until']:
                    sim.real_time_index=rt_state['idx']
                else:
                    rt_state['idx']=int(sim.real_time_index)
                running=bool(sim.run)
                a[13]=1.0 if running else 0.0
                idx=min(max(rt_state['idx'],0),len(mujoco.viewer.PERCENT_REALTIME)-1)
                a[14]=100.0/float(mujoco.viewer.PERCENT_REALTIME[idx])
                a[16]=float(sim.ctrl_noise_std)
                a[17]=float(sim.ctrl_noise_rate)
                # Detect UI Reset / Load key / History scrub / Paused joint slider changes made by RenderLoop
                if last_qpos is not None:
                    if data.time==0.0 and last_time>0.0 and np.allclose(data.qpos,model.qpos0):
                        data.qpos[:]=default_qpos;data.qvel[:]=default_qvel;data.time=0.0
                        mujoco.mj_forward(model,data)
                        last_qpos=data.qpos.copy();last_qvel=data.qvel.copy();last_time=0.0
                        a[20]=1;a[19]=0.0;a[18]+=1
                    elif not (np.array_equal(data.qpos,last_qpos) and np.array_equal(data.qvel,last_qvel)):
                        a[ovr_offset:ovr_offset+nq]=data.qpos
                        a[ovr_offset+nq:ovr_offset+state_len]=data.qvel
                        a[19]=float(data.time);a[20]=2;a[18]+=1
                        last_qpos=data.qpos.copy();last_qvel=data.qvel.copy();last_time=float(data.time)
                if running and int(a[21])==int(a[18]):
                    if has_snap:
                        data.qpos[:]=snapshot[:nq];data.qvel[:]=snapshot[nq:]
                        data.time=max(simtime,1e-4)
                        mujoco.mj_forward(model,data)
                        sim.add_to_history()
                        last_qpos=data.qpos.copy();last_qvel=data.qvel.copy();last_time=float(data.time)
                else:
                    mujoco.mj_forward(model,data)
                    last_qpos=data.qpos.copy();last_qvel=data.qvel.copy();last_time=float(data.time)
                # 3D joint heatmap glow/pulse on links exhibiting HF jitter or twitching
                model.geom_rgba[:]=base_geom_rgba
                j_sev=a[jitter_offset:jitter_offset+nu]
                pulse_fast=0.5+0.5*np.sin(2.0*np.pi*10.0*now)
                pulse_warn=0.5+0.5*np.sin(2.0*np.pi*5.0*now)
                for j,geoms in enumerate(joint_geom_ids):
                    if not geoms:continue
                    sev=float(j_sev[j])
                    if sev>=0.55:
                        col=np.array([1.0,0.10+0.35*pulse_fast,0.06,1.0],dtype=np.float32)
                        blend=0.72+0.28*pulse_fast
                        for g in geoms:
                            model.geom_rgba[g]=(1.0-blend)*base_geom_rgba[g]+blend*col
                    elif sev>=0.20:
                        col=np.array([1.0,0.82,0.08,1.0],dtype=np.float32)
                        blend=min(0.85,max(0.25,(sev-0.20)/0.35*(0.55+0.30*pulse_warn)))
                        for g in geoms:
                            model.geom_rgba[g]=(1.0-blend)*base_geom_rgba[g]+blend*col
                data.xfrc_applied[:]=0
                mujoco.mjv_applyPerturbForce(model,data,pert)
                a[frc_offset:ovr_offset]=data.xfrc_applied.ravel();a[3]=now
                data.xfrc_applied[0,0]=1e-300
                cur_simtime=float(data.time)

            if rendered_frames>=2 and now-last_text_time>=0.1 and not sim.exitrequest and a[0]:
                state={0:'STOP',1:'ZERO',2:'SDK-A',3:'WALK',4:'FIX'}.get(int(a[9]),'Waiting for SDK')
                hang_str=f'ON (z={a[12]:.2f}m, k={a[15]:.0f}N/m)' if a[11]>0.5 else 'OFF (KP+/Up/F9 to hang)'
                flt_str=f'ON (alpha={a[22]:.2f})' if a[22]>0.0 else 'OFF (RAW)'
                j_score=float(a[23]);t_score=float(a[24]);p_freq=float(a[25])
                p_idx=int(a[26]) if 0<=int(a[26])<len(joint_names) else 0
                act_n=int(a[27])
                p_name=joint_names[p_idx] if joint_names else 'none'
                peak_sev=max(j_score,t_score)
                j_pct=min(100.0,j_score*100.0);t_pct=min(100.0,t_score*100.0)
                if peak_sev>=0.55:
                    banner_title='!! CRITICAL: HF JITTER / TWITCH !!'
                    banner_body=f'PEAK: {p_name} ({p_freq:.1f} Hz) | Jitter {j_pct:.0f}% | Twitch {t_pct:.0f}% | Joints {act_n}'
                    banner_scale=int(mujoco.mjtFontScale.mjFONTSCALE_150)
                elif peak_sev>=0.25:
                    banner_title='! WARN: JITTER / TWITCH DETECTED !'
                    banner_body=f'PEAK: {p_name} ({p_freq:.1f} Hz) | Jitter {j_pct:.0f}% | Twitch {t_pct:.0f}% | Joints {act_n}'
                    banner_scale=int(mujoco.mjtFontScale.mjFONTSCALE_150)
                else:
                    banner_title='JITTER MONITOR'
                    banner_body=f'OK (chatter {j_pct:.1f}%, twitch {t_pct:.1f}%)'
                    banner_scale=int(mujoco.mjtFontScale.mjFONTSCALE_100)
                sim.set_texts([
                    (
                        int(mujoco.mjtFontScale.mjFONTSCALE_100),
                        int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                        'Simulation time\nReal-time factor\nPhysics steps (measured)\nPhysics rate (configured)\nSDK control (measured)\nFSM compute (mean)\nViewer FPS\nFSM state\nAction filter\nSuspension\nHF Jitter / Twitch',
                        f'{cur_simtime:.1f} s\n{a[6]:.2f} x\n{a[6]/a[10]:.0f} Hz\n{1/a[10]:.0f} Hz\n{a[7]:.1f} Hz\n{a[8]:.2f} ms\n{a[4]:.1f}\n{state}\n{flt_str}\n{hang_str}\n{j_pct:.0f}% / {t_pct:.0f}% ({act_n} jnt)',
                    ),
                    (
                        banner_scale,
                        int(mujoco.mjtGridPos.mjGRID_TOP),
                        banner_title,
                        banner_body,
                    ),
                ])
                hist_jitter[:-1]=hist_jitter[1:];hist_jitter[-1]=min(105.0,j_pct)
                hist_twitch[:-1]=hist_twitch[1:];hist_twitch[-1]=min(105.0,t_pct)
                xs=np.arange(fig_n,dtype=np.float32)
                fig.linepnt[0]=fig_n;fig.linepnt[1]=fig_n
                fig.linedata[0,0:2*fig_n:2]=xs;fig.linedata[0,1:2*fig_n:2]=hist_jitter
                fig.linedata[1,0:2*fig_n:2]=xs;fig.linedata[1,1:2*fig_n:2]=hist_twitch
                vp=sim.viewport
                fw=min(360,max(220,int(vp.width*0.26))) if vp.width>0 else 320
                fh=min(180,max(120,int(vp.height*0.22))) if vp.height>0 else 160
                fleft=max(8,vp.width-fw-12) if vp.width>fw+16 else 8
                sim.set_figures([(mujoco.MjrRect(fleft,12,fw,fh),fig)])
                rendered_frames=0;last_text_time=now

            elapsed=now-began
            if elapsed>=0.5:
                a[4]=frames/elapsed;frames=0;began=now
        if not a[0]:sim.exit()
        a[0]=0

    side_thread=threading.Thread(target=bridge_loop,daemon=True)
    side_thread.start()
    sim.render_loop()
    a[0]=0
    side_thread.join(timeout=2)
    sim.destroy()
if __name__=='__main__':main()
