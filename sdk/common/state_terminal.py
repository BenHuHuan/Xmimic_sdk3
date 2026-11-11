"""Line-based state commands; enqueue through the same path as web controls."""
import math
import queue
import os
import sys
import threading

EVENTS = {'STOP': 'stop', 'ZERO': 'init_pose', 'SDK-A': 'run_policy', 'SDK_A': 'run_policy', 'WALK': 'walk', 'FIX': 'fix', 'FIXSTAND': 'fix', 'STAND': 'fix'}
REF_PRESETS = {'STAND': (50, 240, 'STAND'), 'DANCE': (250, 430, 'DANCE'), 'FULL': (0, 6573, 'FULL')}
FILTER_PRESETS = {'LIGHT': 0.50, 'MED': 0.35, 'DEFAULT': 0.35, 'STRONG': 0.25}

def parse_state_command(text):
    return EVENTS.get(text.strip().upper())

def parse_terminal_command(text):
    parts=text.strip().split()
    if parts and parts[0].upper()=='HANG':
        if len(parts)!=2 or parts[1].upper() not in ('ON','OFF','UP','DOWN','STRONGER','WEAKER'):
            raise ValueError('Usage: HANG ON / OFF / UP / DOWN / STRONGER / WEAKER (simulation only)')
        return 'sim_fix',parts[1].upper()
    if parts and parts[0].upper()=='FILTER':
        if len(parts)==1 or (len(parts)==2 and parts[1].upper()=='TOGGLE'):
            return 'terminal_filter',('toggle',None)
        if len(parts)==2:
            arg=parts[1].upper()
            if arg=='ON':return 'terminal_filter',('on',None)
            if arg=='OFF':return 'terminal_filter',('off',None)
            if arg in FILTER_PRESETS:return 'terminal_filter',('on',FILTER_PRESETS[arg])
            try:alpha=float(parts[1])
            except ValueError:raise ValueError('Usage: FILTER [ON | OFF | TOGGLE | LIGHT | MED | STRONG | <0<alpha<=1>]')
            if not math.isfinite(alpha) or not (0.0<alpha<=1.0):
                raise ValueError('Filter alpha must satisfy 0 < alpha <= 1.0')
            return 'terminal_filter',('off' if alpha>=1.0 else 'on',alpha)
        if len(parts)==3 and parts[1].upper()=='ON':
            try:alpha=float(parts[2])
            except ValueError:raise ValueError('Usage: FILTER ON <0<alpha<=1>')
            if not math.isfinite(alpha) or not (0.0<alpha<=1.0):
                raise ValueError('Filter alpha must satisfy 0 < alpha <= 1.0')
            return 'terminal_filter',('off' if alpha>=1.0 else 'on',alpha)
        raise ValueError('Usage: FILTER [ON | OFF | TOGGLE | LIGHT | MED | STRONG | <0<alpha<=1>]')
    if parts and parts[0].upper()=='REF':
        if len(parts)==2 and parts[1].upper() in REF_PRESETS:
            return 'terminal_ref',REF_PRESETS[parts[1].upper()]
        if len(parts)==3:
            try:start,end=int(parts[1]),int(parts[2])
            except ValueError:raise ValueError('Usage: REF STAND | DANCE | FULL | <start_frame> <end_frame>')
            if start<0 or end<start or end>=6574:
                raise ValueError('Reference frame range must satisfy 0 <= start <= end <= 6573')
            return 'terminal_ref',(start,end,f'{start}..{end}')
        raise ValueError('Usage: REF STAND | DANCE | FULL | <start_frame> <end_frame>')
    if parts and parts[0].upper() == 'VEL':
        if len(parts) not in (3,4): raise ValueError('Usage: VEL vx vy [wz]')
        values=[float(x) for x in parts[1:]]
        if len(values)==2: values.append(0.0)
        if not all(math.isfinite(x) for x in values): raise ValueError('Velocity must be finite')
        return ('terminal_vel',values)
    event=parse_state_command(text)
    if event is None: raise ValueError('Enter STOP / ZERO / FIX / SDK-A / FILTER [ON|OFF|alpha] / REF STAND|DANCE|FULL|start end / WALK / VEL vx vy [wz]')
    return event,2

def start_state_terminal(controller):
    def reader():
        fd=os.environ.get('TG_CONSOLE_FD')
        ui=os.fdopen(os.dup(int(fd)),'w',buffering=1) if fd else sys.stdout
        commands='STOP / ZERO / FIX / SDK-A / FILTER [ON|OFF|alpha] / REF STAND|DANCE|FULL / WALK / VEL vx vy [wz]'
        if getattr(controller,'simulation_fix',None) is not None:
            commands+=' / HANG ON|OFF|UP|DOWN|STRONGER|WEAKER'
        print(commands,file=ui,flush=True)
        while True:
            if sys.stdin.isatty():
                ui.write('xrobot3> ');ui.flush()
            line=sys.stdin.readline()
            if not line:break
            if not line.strip():continue
            try:event,value=parse_terminal_command(line)
            except ValueError as exc:
                print(str(exc),file=ui,flush=True);continue
            if event=='sim_fix':
                callback=getattr(controller,'simulation_fix',None)
                if callback is None:
                    print('HANG is available only in simulation.',file=ui,flush=True)
                else:
                    callback(value)
                    print('HANG '+value+' requested; check the viewer suspension indicator.',file=ui,flush=True)
                continue
            reply=queue.Queue(maxsize=1)
            controller.events_queue.put(('terminal_request',(event,value,reply)))
            try:message=reply.get(timeout=10)
            except queue.Empty:message='Execution not confirmed: the control loop may be waiting for feedback. Check the runtime log.'
            print(message,file=ui,flush=True)
        if fd:ui.close()
    thread = threading.Thread(target=reader, name='state-terminal', daemon=True)
    thread.start()
    return thread
