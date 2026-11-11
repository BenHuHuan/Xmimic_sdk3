"""Simulation-only elastic suspension, inspired by Unitree elastic-band debugging.
No root teleport, joint clamp, or hardware command. Disabled by default.
"""
import numpy as np

class SimSuspension:
    def __init__(self,mass):
        self.mass=mass
        self.enabled=False
        self.target=np.zeros(3)
        self.stiffness=1000.0
        self.damping=1.4*np.sqrt(self.mass*self.stiffness)
    def _enable_at(self,position):
        if not self.enabled:
            self.target=np.array(position,dtype=float).copy()
            self.target[2]=max(self.target[2],1.15)
        self.enabled=True
    def command(self,command,position):
        if command=='TOGGLE':
            command='OFF' if self.enabled else 'ON'
        if command=='STRONGER':
            if not self.enabled:
                self.stiffness=max(self.stiffness,250.0)
                self._enable_at(position)
            else:
                self.stiffness=float(np.clip(self.stiffness*1.25,50,5000))
            self.damping=1.4*np.sqrt(self.mass*self.stiffness)
        elif command=='WEAKER':
            if self.enabled:
                if self.stiffness<=65.0:
                    self.enabled=False
                else:
                    self.stiffness=float(np.clip(self.stiffness*0.8,50,5000))
                    self.damping=1.4*np.sqrt(self.mass*self.stiffness)
        elif command=='OFF':self.enabled=False
        elif command=='ON':
            if self.stiffness<250.0:self.stiffness=1000.0
            self.damping=1.4*np.sqrt(self.mass*self.stiffness)
            self._enable_at(position)
        elif command in ('UP','DOWN'):
            if not self.enabled and command=='UP':
                self._enable_at(position)
            elif self.enabled:
                self.target[2]=np.clip(self.target[2]+(.05 if command=='UP' else -.05),.3,2.5)
    def force(self,position,velocity):
        if not self.enabled:return np.zeros(3)
        force=self.stiffness*(self.target-position)-self.damping*velocity
        # Scale gravity compensation with stiffness so WEAKER genuinely lowers support force
        grav_scale=min(1.0,self.stiffness/1000.0)
        force[2]+=self.mass*9.81*grav_scale
        norm=np.linalg.norm(force)
        return force*min(1.,3*self.mass*9.81/max(norm,1e-9))
