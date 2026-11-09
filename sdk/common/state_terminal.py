"""Line-based state commands; enqueue through the same path as web controls."""
import math
import queue
import os
import sys
import threading

EVENTS = {'STOP': 'stop', 'ZERO': 'init_pose', 'SDK-A': 'run_policy', 'SDK_A': 'run_policy', 'WALK': 'walk'}

def parse_state_command(text):
    return EVENTS.get(text.strip().upper())

def parse_terminal_command(text):
    parts=text.strip().split()
    if parts and parts[0].upper()=='VEL':
        if len(parts) not in (3,4): raise ValueError('Usage: VEL vx vy [wz]')
        values=[float(x) for x in parts[1:]]
        if len(values)==2: values.append(0.0)
        if not all(math.isfinite(x) for x in values): raise ValueError('Velocity must be finite')
        return ('terminal_vel',values)
    event=parse_state_command(text)
    if event is None: raise ValueError('Enter STOP / ZERO / SDK-A / WALK / VEL vx vy [wz]')
    return event,2

def start_state_terminal(controller):
    def reader():
        fd=os.environ.get('TG_CONSOLE_FD')
        ui=os.fdopen(os.dup(int(fd)),'w',buffering=1) if fd else sys.stdout
        print('STOP / ZERO / SDK-A / WALK / VEL vx vy [wz]',file=ui,flush=True)
        while True:
            if sys.stdin.isatty():
                ui.write('xrobot3> ');ui.flush()
            line=sys.stdin.readline()
            if not line:break
            if not line.strip():continue
            try:event,value=parse_terminal_command(line)
            except ValueError as exc:
                print(str(exc),file=ui,flush=True);continue
            reply=queue.Queue(maxsize=1)
            controller.events_queue.put(('terminal_request',(event,value,reply)))
            try:message=reply.get(timeout=10)
            except queue.Empty:message='Execution not confirmed: the control loop may be waiting for feedback. Check the runtime log.'
            print(message,file=ui,flush=True)
        if fd:ui.close()
    thread = threading.Thread(target=reader, name='state-terminal', daemon=True)
    thread.start()
    return thread
