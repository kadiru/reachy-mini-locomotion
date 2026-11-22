"""Local frontal-plane control of a free foot. Targets are head roll (deg) and lateral shift (mm)."""
from collections import deque
import math

import numpy as np

from .geometry import euler, rotation


class ReleasePlane:
    catch_trigger = yaw_trigger = None
    gyro_interval_average = True

    def __init__(self, side, seed, checked_ik, policy=None, excitation=False):
        self.side, self.checked_ik = side,checked_ik
        self.rng = np.random.default_rng(seed)
        self.policy = policy or {}
        self.excitation = excitation
        self.measured_head = np.zeros(2)
        self.omega = np.zeros(3)
        self.pitch = 0.
        self.previous = self.delayed = None
        self.commands = deque([np.zeros(2) for _ in range(3)],maxlen=3)
        self.release_error = None
        self.filtered_roll = None
        self.filtered_rate = None
        self.state = np.zeros(10)
        self.target_action = np.zeros(2)
        if 'model' in self.policy:
            self.A,self.B,self.c,self.K,self.xeq,self.ueq = (np.asarray(self.policy['model'][k]) for k in
                                                             ('A','B','c','K','xeq','ueq'))

    def observe_attitude(self,t,pitch):
        self.pitch = pitch

    def observe_head_state(self,t,pose):
        self.measured_head = self.side*np.array([np.degrees(euler(pose)[0]),1000*pose[1,3]])

    def observe_angular_velocity(self,t,velocity):
        self.omega = np.degrees(velocity)

    def target(self,t,roll):
        noise = 0. if self.excitation else self.policy.get('noise_deg',.05)
        phi,theta = np.radians([roll,self.pitch])
        gyro=self.omega
        rate = self.side*(gyro[0]+math.sin(phi)*math.tan(theta)*gyro[1]+math.cos(phi)*math.tan(theta)*gyro[2])
        position=np.r_[self.side*roll-32.1+self.rng.normal(0,noise),self.measured_head]
        sensed_rate=rate+self.rng.normal(0,2*noise)
        hold=self.policy.get('filter')
        if hold:
            # Fuse the gyro with attitude at the control rate using only past samples.
            first=self.filtered_roll is None
            predicted=position[0] if first else self.filtered_roll+.02*sensed_rate
            self.filtered_roll=predicted+(1-math.exp(-.02/hold['attitude_s']))*(position[0]-predicted)
            self.filtered_rate=sensed_rate if first else self.filtered_rate+(
                1-math.exp(-.02/hold['rate_s']))*(sensed_rate-self.filtered_rate)
            position[0]=self.filtered_roll
            sensed_rate=self.filtered_rate
        velocity=np.r_[sensed_rate,
                       np.zeros(2) if self.previous is None else (position[1:]-self.previous[1:])/.02]
        observation=np.r_[position,velocity]
        self.previous=position
        self.state=np.concatenate((observation,self.commands[1],self.commands[2]))
        if self.excitation:
            if round(t*50)%self.policy['interval']==0:
                self.target_action=self.rng.uniform(-1,1,2)*self.policy['amplitude']
            command=self.target_action
        else:
            p=self.policy
            delayed=observation if self.delayed is None else self.delayed
            prior=np.concatenate((delayed,self.commands[0],self.commands[1]))
            x=self.A@prior+self.B@self.commands[-1]+self.c if self.delayed is not None else self.state
            if p.get('neutral_trim') and self.delayed is not None:
                # Predict deviations around the measured passive trim to avoid an affine model bias.
                x=self.xeq+self.A@(prior-self.xeq)+self.B@(self.commands[-1]-self.ueq)
            error=x-self.xeq
            if self.release_error is None:
                self.release_error = observation[0] + p.get('lookahead',.10)*observation[3]
            # Adapt the initial stroke and its braking phase to the measured release.
            launch = p.get('launch')
            command=self.ueq-p.get('gain',1.)*self.K@error
            if launch:
                duration=launch['duration']
                if t<duration:
                    command=(1-launch['mix'])*command + self.release_error*np.asarray(launch['stroke'])
                elif t<duration+launch['brake_s']:
                    u=(t-duration)/launch['brake_s']
                    command+=self.release_error*(1-u)*np.asarray(launch['brake'])
                command+=np.asarray(launch['offset'])
                command+=np.asarray(launch['damping'])*error[3]

            if hold:
                alpha=1-math.exp(-.02/hold['command_s'])
                command=self.commands[-1]+alpha*(command-self.commands[-1])
            limit=np.array([p.get('rate',600.),p.get('rate',600.)*.65])*.02
            command=self.commands[-1]+np.clip(command-self.commands[-1],-limit,limit)
        self.delayed=observation
        command=np.clip(command,[-38.,-24.],[38.,24.])
        for scale in (1.,.85,.65,.4,0.):
            u=scale*command
            pose=np.eye(4)
            pose[:3,:3]=rotation((math.radians(self.side*u[0]),0.,0.))
            pose[:3,3]=[0.,self.side*u[1]/1000,.177]
            try:
                self.checked_ik(pose)
                break
            except ValueError:
                if scale==0.:
                    raise
        self.commands.append(u)
        return pose,'frontal plane',float(self.side*(observation[0]+32.1))

    def diagnostics(self):
        return dict(plane_state=self.state.tolist(),plane_action=self.commands[-1].tolist())


class ReleaseSelector:
    """Choose once, from the first available attitude and gyro observation."""
    gyro_interval_average = True
    catch_trigger = yaw_trigger = None

    def __init__(self, side, seed, policy, factory):
        self.side, self.policy, self.factory = side, policy, factory
        self.rng = np.random.default_rng(seed+130010)
        self.controller = None
        self.index = None

    def observe_head_state(self, t, pose):
        self.head = pose
        if self.controller:
            self.controller.observe_head_state(t, pose)

    def observe_attitude(self, t, pitch):
        self.pitch = pitch
        if self.controller:
            self.controller.observe_attitude(t, pitch)

    def observe_angular_velocity(self, t, velocity):
        self.gyro = velocity
        if self.controller:
            self.controller.observe_angular_velocity(t, velocity)

    def target(self, t, roll):
        if self.controller is None:
            phi, theta = np.radians([roll,self.pitch])
            rate = np.degrees(self.gyro[0]+np.tan(theta)*(np.sin(phi)*self.gyro[1]+np.cos(phi)*self.gyro[2]))
            projected = self.side*roll+self.rng.normal(0,.05)+self.policy['lead_s']*(
                self.side*rate+self.rng.normal(0,.1))
            self.index = int(np.argmin(np.abs(np.array(self.policy['centers_deg'])-projected)))
            self.controller = self.factory(self.policy['specialists'][self.index])
            self.controller.observe_head_state(t,self.head)
            self.controller.observe_attitude(t,self.pitch)
            self.controller.observe_angular_velocity(t,self.gyro)
        return self.controller.target(t,roll)

    def diagnostics(self):
        return dict(self.controller.diagnostics(),release_bin=self.index)
