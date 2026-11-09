"""Separate GPU viewer; physics never waits for window sync or OpenGL locks."""
from pathlib import Path
import subprocess,sys,time,json
import numpy as np

class RenderBridge:
    def __init__(self,model,output):
        self.path=Path(output)/'viewer_shared.npy'
        self.nq=model.nq;self.nv=model.nv;self.nb=model.nbody
        self.shared=np.lib.format.open_memmap(self.path,mode='w+',dtype=np.float64,shape=(16+self.nq+self.nv+self.nb*6,));self.shared[:]=0;self.shared[0]=1
        self.proc=subprocess.Popen([sys.executable,__file__,str(self.path),str(self.nq),str(self.nv),str(self.nb)])
    def update(self,data,performance=None):
        a=self.shared;a[1]+=1
        a[16:16+self.nq]=data.qpos;a[16+self.nq:16+self.nq+self.nv]=data.qvel
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
        a[9]={'STOP':0,'ZERO':1,'SDK_A':2,'DEXAMP':3}.get(perf.get('state'),-1) if fresh else -1
        a[2]=data.time;a[5]=now;a[1]+=1
    def apply_force(self,data):
        if time.monotonic()-self.shared[3]<.2:data.xfrc_applied[:]=self.shared[16+self.nq+self.nv:].reshape(self.nb,6)
    def is_running(self):return self.proc.poll() is None and bool(self.shared[0])
    def close(self):
        self.shared[0]=0
        try:self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:self.proc.terminate();self.proc.wait(timeout=3)

def main():
    import mujoco,mujoco.viewer
    path=Path(sys.argv[1]);nq,nv,nb=map(int,sys.argv[2:]);a=np.load(path,mmap_mode='r+')
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).parent/'scene_unitree_visual.xml'));data=mujoco.MjData(model)
    # Only render settings, no changes to collision/physical model.
    model.vis.quality.offsamples=0
    while a[5]==0 and a[0]:time.sleep(.01)
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.distance=3.5;viewer.cam.elevation=-25
        deadline=time.monotonic();frames=0;began=deadline
        while a[0] and viewer.is_running():
            seq=a[1]
            if int(seq)%2:continue
            snapshot=a[16:16+nq+nv].copy();simtime=a[2]
            if a[1]!=seq:continue
            data.qpos[:]=snapshot[:nq];data.qvel[:]=snapshot[nq:];data.time=simtime
            mujoco.mj_forward(model,data)
            data.xfrc_applied[:]=0
            with viewer.lock():mujoco.mjv_applyPerturbForce(model,data,viewer.perturb)
            a[16+nq+nv:]=data.xfrc_applied.ravel();a[3]=time.monotonic()
            state={0:'STOP',1:'ZERO',2:'SDK-A',3:'WALK'}.get(int(a[9]),'Waiting for SDK')
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                'Xrobot3 | '+state+'\nSimulation time\nReal-time factor\nPhysics steps (measured)\nPhysics rate (configured)\nSDK control (measured)\nFSM compute (mean)\nViewer FPS',
                '\n'+f'{simtime:.1f} s\n{a[6]:.2f} x\n{a[6]/model.opt.timestep:.0f} Hz\n{1/model.opt.timestep:.0f} Hz\n{a[7]:.1f} Hz\n{a[8]:.2f} ms\n{a[4]:.1f}'))
            viewer.cam.lookat[:]=data.qpos[:3];viewer.sync();frames+=1
            elapsed=time.monotonic()-began
            if elapsed>=0.5:
                a[4]=frames/elapsed;frames=0;began=time.monotonic()
            deadline+=1/60
            delay=deadline-time.monotonic()
            if delay>0:time.sleep(delay)
            elif delay<-.1:deadline=time.monotonic()
    a[0]=0
if __name__=='__main__':main()
