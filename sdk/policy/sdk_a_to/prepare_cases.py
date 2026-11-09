"""Predeclared native test fixtures, including physically settled flat starts."""
from pathlib import Path
import json, hashlib
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

HERE=Path(__file__).resolve().parent
def quat(r): return r.as_quat()[[3,0,1,2]]
def rotation(q):return R.from_quat(np.asarray(q)[[1,2,3,0]])
def make_model(meta):
    model=mujoco.MjModel.from_xml_path(meta['scene'])
    model.opt.timestep=.001
    # Official xSIM numerical options; keep training's integrator and plant.
    model.opt.solver=mujoco.mjtSolver.mjSOL_CG;model.opt.iterations=100
    return model

def main():
    meta=json.loads((HERE/'manifest.json').read_text()); c=json.loads(Path(meta['contract']).read_text())
    model=make_model(meta); d=mujoco.MjData(model);ref=np.load(meta['reference'])
    count=len(ref['joint_pos']); phases=[0,count//4,count//2,3*count//4];cases=[];fixtures={}
    def append(name,kind,qpos,qvel,phase,seconds,**extra):
        fixtures[name+'_qpos']=np.array(qpos);fixtures[name+'_qvel']=np.array(qvel)
        cases.append(dict(id=name,kind=kind,phase=int(phase),seconds=float(seconds),**extra))
    def tracking(frame):
        q=np.r_[ref['root_pos_w'][frame],ref['root_quat_w'][frame],ref['joint_pos'][frame]].astype(float)
        v=np.r_[ref['root_lin_vel_w'][frame],rotation(q[3:7]).inv().apply(ref['root_ang_vel_w'][frame]),ref['joint_vel'][frame]].astype(float)
        return q,v
    for k,phase in enumerate(phases):
        append(f'normal_p{k}','normal',*tracking(phase),phase,count*.02)
    directions=['front','front_left','left','back_left','back','back_right','right','front_right']
    for force in (250,350,600):
        for k,onset in enumerate((8,13,18,23)):
            for i,name in enumerate(directions):
                append(f'push_{name}_{force}_t{onset}','push',*tracking(0),0,onset+30,
                    direction=name,angle_deg=i*45,force_N=force,push_time_s=onset,push_duration_s=.2)
    poses={}
    poses['neutral']=np.array(c['offset'])
    poses['straight']=np.zeros(29);poses['straight'][[16,23]]=[.2,-.2]
    poses['compact']=np.array(c['offset']);poses['compact'][[0,6,3,9,4,10,18,25]]=[-1,-1,1.6,1.6,-.5,-.5,-1,-1]
    poses['asymmetric']=np.array(c['offset']);poses['asymmetric'][[0,6,1,7,3,9,18,25]]=[-.8,-.2,.35,-.12,1.5,.6,-1,-.2]
    orientations={'supine':R.from_euler('y',-90,degrees=True),'prone':R.from_euler('y',90,degrees=True),
                  'left':R.from_euler('x',90,degrees=True),'right':R.from_euler('x',-90,degrees=True)}
    flat_audit=[]
    for side,orient in orientations.items():
        for pose,jq in poses.items():
            mujoco.mj_resetData(model,d);d.qpos[:3]=[0,0,2];d.qpos[3:7]=quat(orient);d.qpos[7:]=jq
            mujoco.mj_forward(model,d);minimum=10.
            for g in range(model.ngeom):
                if model.geom_bodyid[g]==0 or not (model.geom_contype[g] or model.geom_conaffinity[g]):continue
                mid=model.geom_dataid[g]
                if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                    verts=model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
                    minimum=min(minimum,float((verts@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g])[:,2].min()))
            d.qpos[2]-=minimum-.01;mujoco.mj_forward(model,d)
            # Passive settling, no active policy or success selection.
            for _ in range(1000):d.ctrl[:]=0;mujoco.mj_step(model,d)
            mujoco.mj_forward(model,d)
            penetration=max([0.]+[-float(x.dist) for x in d.contact])
            if penetration>.025 or not np.isfinite(d.qpos).all():raise ValueError((side,pose,penetration))
            flat_audit.append(dict(side=side,pose=pose,height=float(d.qpos[2]),max_penetration_m=penetration,
                root_speed=float(np.linalg.norm(d.qvel[:6]))))
            for k,phase in enumerate(phases):
                append(f'flat_{side}_{pose}_p{k}','flat',d.qpos.copy(),d.qvel.copy(),phase,40,
                       orientation=side,limbs=pose,settling_seconds=1.)
    library=Path(meta['scene']).parent.parent/'reference/recovery_library_amp7_bones8_50hz'
    for path in sorted(library.glob('*.npz')):
        m=np.load(path);valid=np.flatnonzero(m['reset_eligible_mask'].astype(bool).reshape(-1))
        frame=int(valid[0]);anchor=m['body_names'].tolist().index('waist_pitch_link')
        yaw=rotation(m['body_quat_w'][-1,anchor]).as_euler('xyz')[2];align=R.from_euler('z',-yaw)
        rp=align.apply(m['root_pos_w'][frame]);ap=align.apply(m['body_pos_w'][frame,anchor]);rp[:2]-=ap[:2]
        rq=align*rotation(m['root_quat_w'][frame]);rv=align.apply(m['root_lin_vel_w'][frame]);av=rq.inv().apply(align.apply(m['root_ang_vel_w'][frame]))
        q=np.r_[rp,quat(rq),m['joint_pos'][frame]];v=np.r_[rv,av,m['joint_vel'][frame]]
        for k,phase in enumerate(phases):
            append(f'ref_{path.stem}_p{k}','reference',q,v,phase,40,clip=path.stem,frame=frame)
    np.savez_compressed(HERE/'fixtures.npz',**fixtures)
    protocol=dict(format='tiangong3_sdk_a_native_v1',cases=cases,flat_fixture_audit=flat_audit,
        policy_checkpoint_sha256=meta['checkpoint_sha256'],fixture_sha256=hashlib.sha256((HERE/'fixtures.npz').read_bytes()).hexdigest(),
        physical_standing=dict(height_m=.8*c['root_pos'][2],pelvis_up_min=.8,continuous_seconds=2),
        route_tracking='standing AND tracking_weight>0.5 for 2 seconds; routing proxy only',
        fall='non-foot ground contact AND pelvis height <0.65 nominal OR pelvis_up<0.5, after push',
        failures='observe whole window, no early reset, never count no-fall as fall recovery',
        backend='Deploy_Tienkung 3.0 official RobotInterface -> ROS2 FastDDS -> xSIM RobotSimulator adapter',
        adaptations=['new URDF body29 headfixed matching checkpoint','50Hz actor','IMU mounting calibration',
            'training PD/effort, no command angle clip','1000Hz PD using current encoder q/dq',
            'headless eval, disabled startup band, 400Hz measured-state publish'],
        plant=dict(dt=.001,solver='CG',iterations=100,ls_iterations=int(model.opt.ls_iterations),integrator='implicitfast'),
        hardware=False)
    (HERE/'protocol.json').write_text(json.dumps(protocol,indent=2))
    print({kind:sum(x['kind']==kind for x in cases) for kind in ('normal','push','flat','reference')})
if __name__=='__main__':main()
