"""xMIGCS dex3 SDK-A FSM -> FastDDS -> Dknt0/xmujoco. Separate OS processes.

Uses xmigcs_dex3 motor serialization and xmujoco RobotSimulator command handling,
PVD and status serialization. The adapter aligns embodiment, sensors and actor rate.
No simulator state enters the controller except official joint/IMU messages.
"""
from pathlib import Path
import argparse, json, os, sys, time, threading, queue, subprocess
import numpy as np
import itertools
from collections import deque

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
DEPLOY=HERE.parents[1];PROJECT=DEPLOY.parent;XSIM=PROJECT/'xmujoco/scripts'
sys.path[:0]=[str(DEPLOY),str(XSIM),str(HERE)]

def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
def qmul(a,b):
    w,x,y,z=a;v,i,j,k=b
    return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v])
def qrot(q):
    import mujoco
    r=np.empty(9);mujoco.mju_quat2Mat(r,q);return r.reshape(3,3)
def spin(node):
    import rclpy
    while rclpy.ok():rclpy.spin_once(node,timeout_sec=.01)

def controller(args):
    import rclpy, onnxruntime as ort
    from rclpy.node import Node
    from std_msgs.msg import String
    from common.robot_interface import RobotInterfaceImpl
    from common.robot_data import RobotData
    from common.joystick import ControlFlag
    from FSM.fsm_base import FSMStateName
    meta=json.loads((HERE/'manifest.json').read_text());meta.update(contract=str(HERE/'deploy.json'),scene=str(PROJECT/meta['scene']));contract=json.loads(Path(meta['contract']).read_text())
    rclpy.init();node=Node('tiangong_sdk_a_controller')
    data=RobotData();data.control_dt_=0.02;interface=RobotInterfaceImpl(data,str(DEPLOY/'local_setup/sim_workspace/config/sdk_a.yaml'))
    interface.init(node);assert interface.sim;interface.current_state=FSMStateName.SDK_A
    from FSM.robot_fsm import get_robot_fsm
    fsm=get_robot_fsm(data,{"sdk_a_only":True});skill=fsm.state_objects[FSMStateName.SDK_A]
    data.joint_kp_p_[:]=contract['kp'];data.joint_kd_p_[:]=contract['kd']
    mount=np.array(meta['imu_mount_quat']);inverse=mount*[1,-1,-1,-1];R=qrot(mount)
    pub=node.create_publisher(String,'/sdk_a/telemetry',10)
    state={'case':None,'counter':0,'reset':True,'history':np.zeros((1,884),np.float32),'raw':np.zeros((1,29),np.float32)}
    def case_callback(msg):
        obj=json.loads(msg.data)
        if state['case'] is None or obj['id']!=state['case']['id'] or obj['running']!=state['case']['running']:
            state.update(case=obj,counter=0,reset=True,history=np.zeros((1,884),np.float32),raw=np.zeros((1,29),np.float32))
    node.create_subscription(String,'/sdk_a/case',case_callback,10)
    last_time=None
    def tick():
        nonlocal last_time
        case=state['case']
        if case is None:return
        queues=[interface.queue_leg_motor_state,interface.queue_waist_motor_state,interface.queue_arm_motor_state,interface.queue_imu_xsens]
        try:msgs=[q.peek() for q in queues]
        except queue.Empty:return
        now=time.time();ages=[now-stamp(m) for m in msgs]
        if max(ages)>.1:return
        interface.update_robot_state()
        np.copyto(data.q_a_s_,data.q_a_p_);np.copyto(data.q_dot_a_s_,data.q_dot_a_p_)
        imu=msgs[-1]
        raw_quat=np.array([imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z])
        pelvis=qmul(raw_quat,inverse);pelvis/=np.linalg.norm(pelvis)
        gyro=R@np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        acc=R@np.array([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z])
        if state['reset']:skill.on_enter()
        skill.phase=case['phase'];skill.running=case['running']
        data.sdk_a_pelvis_quat=pelvis;data.sdk_a_gyro=gyro;data.sdk_a_acc=acc
        frame=case['phase']+state['counter']
        sim_flag=ControlFlag();sim_flag.fsm_state_command='gotoSDK_A'
        fsm.run_fsm(sim_flag)
        target=skill.target[None];raw=skill.output_raw[None];hist=skill.history;weights=skill.weights[None]
        interface.send_motor_commands(ControlFlag())
        dt=None if last_time is None else time.monotonic()-last_time;last_time=time.monotonic()
        obj=dict(id=case['id'],running=case['running'],counter=state['counter'],phase=frame,
            wall_time=now,weights=weights[0].tolist(),target=target[0].tolist(),raw=raw[0].tolist(),
            sensor_age_s=max(ages),sensor_group_skew_s=max(map(stamp,msgs))-min(map(stamp,msgs)),policy_interval_s=dt)
        if getattr(skill,'protection_telemetry',None) is not None:
            obj['protection']=skill.protection_telemetry
        pub.publish(String(data=json.dumps(obj)))
        if case['running']:
            state.update(counter=state['counter']+1,reset=False,history=hist,raw=raw)
    node.create_timer(.02,tick)
    print('xMIGCS SDK_A FSM ready; SDK-A TO ONNX; serial simulation ankles; 50 Hz',flush=True)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

def simulator(args):
    import mujoco, rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from bodyctrl_msgs.msg import CmdMotorCtrl,MotorStatusMsg,Imu
    from simulator_view_asyn import RobotSimulator
    from common.body_id_map import BodyServoIdMap
    from prepare_cases import make_model
    meta=json.loads((HERE/'manifest.json').read_text());meta.update(contract=str(HERE/'deploy.json'),scene=str(PROJECT/meta['scene']));contract=json.loads(Path(meta['contract']).read_text())
    protocol=json.loads((HERE/'protocol.json').read_text());fixtures=np.load(HERE/'fixtures.npz')
    cases=protocol['cases']
    if args.cases:cases=[c for c in cases if c['id'] in args.cases.split(',')]
    cases=[c for i,c in enumerate(cases) if i%args.workers==args.worker]
    if args.push_force is not None:
        if args.push_force <= 0: raise ValueError('Push force must be positive')
        cases=[dict(c, fixture_id=c['id'], force_N=args.push_force,
                    id=c['id'].replace(f"_{c['force_N']}_",f"_{args.push_force:g}_"))
               if c['kind']=='push' else c for c in cases]
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('tiangong_xsim_sdk_a')
    class AlignedSimulator(RobotSimulator):
        def __init__(self):
            # Keep upstream XML/source untouched; body/asset/PD contract comes from checkpoint.
            self.node=node;self.debug=False
            display_meta=dict(meta,scene=str(HERE/'scene_unitree_visual.xml')) if args.interactive else meta
            self.model=make_model(display_meta);self.data=mujoco.MjData(self.model)
            if args.compute=='training':
                self.model.opt.timestep=.002;self.model.opt.solver=mujoco.mjtSolver.mjSOL_NEWTON;self.model.opt.iterations=10
            self.joint_names=contract['joint_names'];ids=BodyServoIdMap();ids.body_can_id_map_init()
            self.motor_id_to_joint={ids.get_id_by_index(i):name for i,name in enumerate(self.joint_names)}
            self.joint_to_motor_id={v:k for k,v in self.motor_id_to_joint.items()}
            self.motor_actuator_indices_map={f'motor_{n}':self.model.actuator(n).id for n in self.joint_names}
            self.indices={n:i for i,n in enumerate(self.joint_names)}
            self.leg_joint_names=self.joint_names[:12];self.waist_joint_names=self.joint_names[12:15];self.arm_joint_names=self.joint_names[15:]
            self.joint_cmd_lock=threading.Lock();self.sensor_data_lock=threading.Lock()
            self.raw_joint_commands=queue.Queue(maxsize=1024);self.last_raw_joint_commands={};self.joint_commands={}
            self.last_sensor_data=None;self.telemetry={};self.performance={};self.group_count={'leg':0,'waist':0,'arm':0}
            self.model.opt.timestep=.001
            self.model.opt.solver=mujoco.mjtSolver.mjSOL_NEWTON
            self.model.opt.iterations=100;self.model.opt.ls_iterations=50
            self.model.opt.tolerance=1e-8
            for group in ('leg','waist','arm'):
                setattr(self,group+'_status_pub',node.create_publisher(MotorStatusMsg,f'/{group}/status',10))
                def callback(msg,group=group):
                    self.group_count[group]+=1;self.joint_cmd_callback(msg)
                node.create_subscription(CmdMotorCtrl,f'/{group}/cmd_ctrl',callback,10)
            self.imu_status_pub=node.create_publisher(Imu,'/imu/status',10)
            node.create_subscription(String,'/sdk_a/telemetry',lambda m:setattr(self,'telemetry',json.loads(m.data)),10)
            node.create_subscription(String,'/sdk_a/performance',lambda m:setattr(self,'performance',json.loads(m.data)),10)
            self.fix_requests=queue.Queue()
            node.create_subscription(String,'/xrobot3/sim_fix',lambda m:self.fix_requests.put(m.data),10)
            self.case_pub=node.create_publisher(String,'/sdk_a/case',10)
            self.mount=np.array([1.,0.,0.,0.] if args.standard_sdk else meta['imu_mount_quat']);self.mount_R=qrot(self.mount)
            self.acc_sensor_id=self.model.sensor('imu_in_pelvis_acc').id if any(self.model.sensor(i).name=='imu_in_pelvis_acc' for i in range(self.model.nsensor)) else -1
            self.cached_q=np.zeros(29);self.cached_v=np.zeros(29)
        def get_status(self,name):
            i=self.indices[name]
            if args.pd_feedback=='legacy100':return dict(position=self.cached_q[i],velocity=self.cached_v[i],torque=0.)
            return dict(position=self.data.qpos[7+i],velocity=self.data.qvel[6+i],torque=0.)
        def capture_sensors(self):
            d=self.data
            self.cached_q=d.qpos[7:].copy();self.cached_v=d.qvel[6:].copy()
            pelvis_acc=d.sensor(self.acc_sensor_id).data.astype(float) if self.acc_sensor_id>=0 else np.zeros(3)
            self.last_sensor_data=dict(joint_positions=dict(zip(self.joint_names,d.qpos[7:].astype(float))),
                joint_velocities=dict(zip(self.joint_names,d.qvel[6:].astype(float))),
                joint_torques=dict(zip(self.joint_names,d.actuator_force.astype(float))),
                imu_orientation=qmul(d.qpos[3:7],self.mount),
                imu_angular_velocity=self.mount_R.T@d.qvel[3:6],imu_linear_acceleration=self.mount_R.T@pelvis_acc)
        def publish(self):
            for group in ('leg','waist','arm'):
                self._publish_joint_group(getattr(self,group+'_joint_names'),getattr(self,group+'_status_pub'),self.last_sensor_data)
            self._publish_imu_status(self.imu_status_pub,self.last_sensor_data)
        def signal(self,case,running):self.case_pub.publish(String(data=json.dumps(dict(id=case['id'],phase=case['phase'],running=running))))
    sim=AlignedSimulator();model=sim.model;d=sim.data
    thread=threading.Thread(target=spin,args=(node,),daemon=True);thread.start()
    viewer=None;render_bridge=None
    if args.interactive:
        from render_bridge import RenderBridge
        render_bridge=RenderBridge(model,args.output)
    if args.viewer and not args.interactive:
        import mujoco.viewer
        viewer=mujoco.viewer.launch_passive(model,d);viewer.cam.distance=3.5;viewer.cam.elevation=-25
    from sim_suspension import SimSuspension
    from jitter_monitor import JitterMonitor
    suspension=SimSuspension(float(model.body_mass.sum()))
    suspension_body=model.body('waist_pitch_link').id
    dt=float(model.opt.timestep);substeps=round(.02/dt)
    jitter_monitor=JitterMonitor(sim.joint_names,contract['effort'],dt=dt)
    try:
        for case in cases:
            dest=args.output/case['id'];dest.mkdir(exist_ok=True)
            if (dest/'result.json').exists():continue
            fixture_id=case.get('fixture_id',case['id'])
            mujoco.mj_resetData(model,d);d.qpos[:]=fixtures[fixture_id+'_qpos'];d.qvel[:]=fixtures[fixture_id+'_qvel']
            mujoco.mj_forward(model,d)
            jitter_monitor.reset()
            sim.last_raw_joint_commands.clear();sim.telemetry={};sim.group_count={g:0 for g in sim.group_count}
            with sim.joint_cmd_lock:
                while not sim.raw_joint_commands.empty():sim.raw_joint_commands.get_nowait()
            sim.capture_sensors();started=time.monotonic();ready=False
            while time.monotonic()-started<20:
                sim.signal(case,False);sim.publish();sim.set_motor_cmd()
                if (args.standard_sdk or sim.telemetry.get('id')==case['id']) and len(sim.last_raw_joint_commands)==29:
                    ready=True;break
                time.sleep(.02)
            if not ready:raise RuntimeError('DDS startup incomplete: '+case['id'])
            sim.signal(case,True)
            records=deque(maxlen=3000) if args.interactive else [];began=time.monotonic();deadline=began;last_pub=began
            nominal=contract['root_pos'][2];seconds=args.seconds or case['seconds']
            push_vector=np.zeros(3)
            if case['kind']=='push':
                angle=np.deg2rad(case['angle_deg']);direction=np.array([np.cos(angle),np.sin(angle),0.])
            for step in (itertools.count() if args.interactive else range(round(seconds/dt))):
                if args.interactive and not render_bridge.is_running():break
                if args.interactive and render_bridge.poll_state_override(model,d,fixtures[fixture_id+'_qpos'],fixtures[fixture_id+'_qvel']):
                    jitter_monitor.reset()
                    sim.capture_sensors();sim.publish();deadline=time.monotonic()
                if args.interactive and render_bridge.is_paused():
                    now=time.monotonic()
                    if now-last_pub>=.01:
                        sim.capture_sensors();sim.publish();last_pub=now
                    for command in render_bridge.read_commands():sim.fix_requests.put(command)
                    while not sim.fix_requests.empty():
                        suspension.command(sim.fix_requests.get_nowait(),d.xpos[suspension_body])
                    render_bridge.shared[11]=float(suspension.enabled)
                    render_bridge.shared[12]=suspension.target[2]
                    render_bridge.shared[15]=suspension.stiffness
                    render_bridge.update(d, sim.performance, jitter_monitor=jitter_monitor)
                    deadline=time.monotonic()
                    time.sleep(0.01)
                    continue
                if step%max(1,round(.01/dt))==0:sim.capture_sensors()
                # Async low-level loop; fresh encoders for PD every 1 ms. DDS status is separate.
                now=time.monotonic()
                if now-last_pub>=.0025:
                    sim.capture_sensors();sim.publish();last_pub=now
                sim.set_motor_cmd();d.xfrc_applied[:]=0
                if args.interactive:
                    render_bridge.apply_ctrl_noise(model,d)
                    render_bridge.apply_force(d)
                t=step*dt
                if case['kind']=='push' and case['push_time_s']<=t<case['push_time_s']+.2:
                    if not np.any(push_vector):
                        yaw=np.arctan2(2*(d.qpos[3]*d.qpos[6]+d.qpos[4]*d.qpos[5]),1-2*(d.qpos[5]**2+d.qpos[6]**2))
                        push_vector=case['force_N']*np.array([np.cos(yaw)*direction[0]-np.sin(yaw)*direction[1],np.sin(yaw)*direction[0]+np.cos(yaw)*direction[1],0])
                    d.xfrc_applied[model.body('waist_pitch_link').id,:3]=push_vector
                if render_bridge and step%10==0:
                    for command in render_bridge.read_commands():sim.fix_requests.put(command)
                while not sim.fix_requests.empty():
                    suspension.command(sim.fix_requests.get_nowait(),d.xpos[suspension_body])
                if suspension.enabled:
                    velocity=np.zeros(6)
                    mujoco.mj_objectVelocity(model,d,mujoco.mjtObj.mjOBJ_BODY,suspension_body,velocity,0)
                    d.xfrc_applied[suspension_body,:3]+=suspension.force(d.xpos[suspension_body],velocity[3:])
                mujoco.mj_step(model,d)
                if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite state '+case['id'])
                jitter_monitor.update(d.qvel[6:],d.actuator_force)
                if step%substeps==0:
                    tele=sim.telemetry
                    ground=False;maxforce=0.;penetration=0.
                    for ct in d.contact:
                        b=[int(model.geom_bodyid[ct.geom1]),int(model.geom_bodyid[ct.geom2])]
                        if 0 in b:
                            other=b[1] if b[0]==0 else b[0]
                            ground|=model.body(other).name not in ('ankle_roll_l_link','ankle_roll_r_link')
                        penetration=max(penetration,-float(ct.dist))
                    up=qrot(d.qpos[3:7])[2,2]
                    record=dict(t=t+dt,wall=time.monotonic()-began,qpos=d.qpos.copy(),qvel=d.qvel.copy(),
                        torque=d.actuator_force.copy(),target=np.array(tele.get('target',contract['offset'])),
                        weights=np.array(tele.get('weights',[np.nan,np.nan])),phase=tele.get('phase',case['phase']),
                        height=float(d.qpos[2]),up=float(up),nonfoot_ground=ground,
                        penetration=penetration,sensor_age=tele.get('sensor_age_s',np.nan),
                        group_skew=tele.get('sensor_group_skew_s',np.nan),policy_interval=tele.get('policy_interval_s') or np.nan,
                        counter=tele.get('counter',-1),telemetry_age=time.time()-tele.get('wall_time',time.time()))
                    records.append(record)
                    if step%1000==0:
                        status=dict(suspension_stiffness=suspension.stiffness,suspension_enabled=suspension.enabled,suspension_target_height=float(suspension.target[2]),physics_dt_s=dt,physics_hz=1/dt,performance=sim.performance,sim_s=float(d.time),wall_s=time.monotonic()-began,height=float(d.qpos[2]),up=float(up),gate=record['weights'].tolist(),policy_counter=record['counter'],sensor_age_s=record['sensor_age'],policy_dt_s=record['policy_interval'],motor_count=len(sim.last_raw_joint_commands),solver_iterations=int(model.opt.iterations),ls_iterations=int(model.opt.ls_iterations),jitter=jitter_monitor.summary())
                        if render_bridge:status.update(viewer_fps=float(render_bridge.shared[4]),render_separate_process=True)
                        (args.output/'live_status.json').write_text(json.dumps(status))
                    if viewer and not args.interactive and step%(substeps*2)==0:viewer.cam.lookat[:]=d.qpos[:3];viewer.sync()
                if args.interactive and step%2==0:
                    render_bridge.shared[11]=float(suspension.enabled)
                    render_bridge.shared[12]=suspension.target[2]
                    render_bridge.shared[15]=suspension.stiffness
                    render_bridge.update(d, sim.performance, jitter_monitor=jitter_monitor)
                deadline+=dt*(render_bridge.slowdown() if args.interactive else 1.0)
                remaining=deadline-time.monotonic()
                if remaining>0:time.sleep(remaining)
                elif remaining<-.1:deadline=time.monotonic()
            if args.interactive:return
            trace={key:np.asarray([r[key] for r in records]) for key in records[0]}
            np.savez_compressed(dest/'trace.npz',**trace)
            result=summarize(case,trace,nominal)
            result.update(wall_seconds=time.monotonic()-began,sim_seconds=float(d.time),pd_feedback=args.pd_feedback,
                compute=args.compute,command_message_counts=sim.group_count,controller_external_process=True,
                plant=dict(dt=dt,solver=int(model.opt.solver),iterations=int(model.opt.iterations),ls_iterations=int(model.opt.ls_iterations)),
                jitter=jitter_monitor.summary(),
                checkpoint_sha256=meta['checkpoint_sha256'])
            (dest/'result.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(result),flush=True)
    finally:
        if viewer:viewer.close()
        if render_bridge:render_bridge.close()
        rclpy.shutdown();thread.join(timeout=2);node.destroy_node()

def summarize(case,tr,nominal):
    t=tr['t'];standing=(tr['height']>.8*nominal)&(tr['up']>.8)
    tracking=standing&(tr['weights'][:,0]>.5)
    fallen=tr['nonfoot_ground']&((tr['height']<.65*nominal)|(tr['up']<.5))
    onset=case.get('push_time_s',0.)
    falls=np.flatnonzero(fallen&(t>=onset))
    first_fall=None if len(falls)==0 else float(t[falls[0]])
    def first_run(mask,start):
        count=0
        for i,ok in enumerate(mask):
            count=count+1 if ok and t[i]>=start else 0
            if count>=100:return float(t[i]-start)
        return None
    start=first_fall if case['kind']=='push' and first_fall is not None else onset
    s=first_run(standing,start);r=first_run(tracking,start)
    interval=tr['policy_interval'];interval=interval[np.isfinite(interval)]
    return dict(case=case,physical_fall=first_fall is not None,first_fall_s=first_fall,
        standing_confirm_after_fall_s=s,tracking_confirm_after_fall_s=r,
        ever_standing=s is not None,ever_tracking=r is not None,
        end_standing=bool(standing[-100:].all()),end_tracking=bool(tracking[-100:].all()),
        max_height_m=float(tr['height'].max()),final_height_m=float(tr['height'][-1]),
        recovery_weight_mean=float(tr['weights'][:,1].mean()),
        max_penetration_m=float(tr['penetration'].max()),
        policy_dt_median_s=float(np.median(interval)) if len(interval) else None,
        policy_dt_p99_s=float(np.quantile(interval,.99)) if len(interval) else None,
        telemetry_age_p99_s=float(np.quantile(tr['telemetry_age'],.99)),
        unique_policy_steps=int(len(set(tr['counter'].tolist()))))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('role',choices=['controller','simulator','pair'])
    p.add_argument('--output',type=Path,default=ROOT/'output/native_20261102')
    p.add_argument('--cases');p.add_argument('--worker',type=int,default=0);p.add_argument('--workers',type=int,default=1)
    p.add_argument('--seconds',type=float);p.add_argument('--viewer',action='store_true')
    p.add_argument('--push-force',type=float,help='Override push magnitude; preserve source fixture and label actual force')
    p.add_argument('--pd-feedback',choices=['current','legacy100'],default='current')
    p.add_argument('--compute',choices=['native','training'],default='native')
    p.add_argument('--interactive',action='store_true')
    p.add_argument('--standard-sdk',action='store_true',help='Run the same rl_control_node as hardware')
    a=p.parse_args()
    if a.interactive:
        a.viewer=True;a.cases='normal_p0'
        _m=json.loads((HERE/'manifest.json').read_text())
        print(f"INTERACTIVE: Tiangong SDK-A {_m.get('training_run','TO 30k')} [{_m['checkpoint_sha256'][:12]}] | xMIGCS -> ROS2/FastDDS -> xmujoco | Newton100 LS50 1000Hz | no automatic push | double-click torso, Ctrl+right drag | close window to exit",flush=True)
    if os.getenv('ROS_LOCALHOST_ONLY')!='1' and not a.standard_sdk:raise RuntimeError('Legacy evaluation uses localhost-only DDS')
    if a.role=='controller':controller(a)
    elif a.role=='simulator':simulator(a)
    else:
        a.output.mkdir(parents=True,exist_ok=True)
        with (a.output/f'controller_{a.worker}.log').open('w') as logfile:
            controller_cmd=[sys.executable,str(DEPLOY/'rl_control_node.py')] if a.standard_sdk else [sys.executable,str(__file__),'controller']
            console_fds=(int(os.environ['TG_CONSOLE_FD']),) if 'TG_CONSOLE_FD' in os.environ else ()
            child=subprocess.Popen(controller_cmd,stdout=subprocess.PIPE if a.standard_sdk else logfile,stderr=subprocess.STDOUT,text=True,pass_fds=console_fds)
            if a.standard_sdk:
                def relay_output():
                    for line in child.stdout:
                        logfile.write(line);logfile.flush()
                        print(line,end='',flush=True)
                relay=threading.Thread(target=relay_output,daemon=True);relay.start()
            try:simulator(a)
            finally:
                child.terminate()
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                if a.standard_sdk:relay.join(timeout=2)

if __name__=='__main__':main()
