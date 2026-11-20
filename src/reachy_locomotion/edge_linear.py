"""Local identified edge regulator using simulated IMU and head pose sensing."""
from collections import deque
import math
import numpy as np

from .edge_balance import EdgeBalanceController
from .geometry import euler,rotation
from .primitives import blend


class LinearEdgeController(EdgeBalanceController):
    def __init__(self,recipe,feedback,side,checked_ik,linear=None,excitation_seed=None):
        super().__init__(recipe,feedback,side,checked_ik)
        self.linear=linear;self.head=np.zeros(4);self.previous_observation=None;self.state=np.zeros(20)
        count=self.f.sensor_delay_frames+2
        self.command_history=deque([np.zeros(4) for _ in range(count)],maxlen=count)
        self.sensor_history=deque([],maxlen=self.f.sensor_delay_frames+1)
        self.excitation_rng=np.random.default_rng(excitation_seed);self.excitation=excitation_seed is not None
        self.random_target=np.zeros(4);self.current_action=np.zeros(4);self.observed=np.zeros(12)
        self.control_gain=1.
        self.tracking_error_limit=0.;self.command_rate_limit=0.
        self.pitch_trigger=None
        if linear is not None:
            self.A=np.array(linear['A']);self.B=np.array(linear['B']);self.c=np.array(linear['c'])
            self.K=np.array(linear['gain']);self.xeq=np.array(linear['x_equilibrium']);self.ueq=np.array(linear['u_equilibrium'])
            for value,shape in [(self.A,(20,20)),(self.B,(20,4)),(self.c,(20,)),(self.K,(4,20)),(self.xeq,(20,)),(self.ueq,(4,))]:
                if value.shape!=shape or not np.isfinite(value).all():raise ValueError('Invalid learned edge model')

    def observe_head_state(self,t,pose):
        self.head=np.r_[np.degrees(euler(pose)[:2]),1000*pose[:2,3]]

    def target(self,t,roll_deg):
        # Signed roll and head roll/y are mirrored; pitch/x remain unchanged.
        reference=self.linear.get('reference_roll_deg',32.1) if self.linear is not None else self.f.target_roll_deg
        h=self.head*np.array([self.side,1,1,self.side]);position=np.r_[self.side*roll_deg-reference,self.pitch,h]
        noise=self.rng.normal(0,self.f.sensor_noise_deg,2);position[:2]+=noise
        self._sensor_noise_sample=noise*np.array([self.side,1])
        velocity=np.zeros(6) if self.previous_observation is None else (position-self.previous_observation)/.02
        self.previous_observation=position.copy();self.observed=np.r_[position,velocity]
        self.sensor_history.append(self.observed.copy())
        self.state=np.r_[self.sensor_history[0],self.command_history[0],self.command_history[1]]
        if self.linear is not None:
            # Predict across the timestamped sensor age using only the learned
            # nominal model and our own past commands, not hidden plant state.
            for action in list(self.command_history)[2:]:self.state=self.A@self.state+self.B@action+self.c
        projected_before=self.projected_frames
        pose,stage,sensed=super().target(t,roll_deg)
        if self.catch_trigger is not None:
            self.projected_frames=projected_before;self.scale=1.
            if self.excitation:
                if round(t/.02)%5==0:self.random_target=self.excitation_rng.normal(0,[2.,2.,1.,1.])
                action=self.random_target
            elif self.linear is not None:
                action=self.ueq-self.control_gain*self.K@(self.state-self.xeq)
            else:action=np.zeros(4)
            if self.tracking_error_limit:
                limits=self.tracking_error_limit*np.array([1,1,.5,.5])
                action=np.clip(action,self.state[2:6]-limits,self.state[2:6]+limits)
            action=np.clip(action,[-40,-40,-25,-25],[40,40,25,25])
            if not self.hold_from_start:
                a=np.degrees(euler(self.capture_pose));origin=np.array([self.side*a[0],a[1],1000*self.capture_pose[0,3],self.side*1000*self.capture_pose[1,3]])
                u=blend((t-self.catch_trigger)/self.f.capture_blend_s);action=(1-u)*origin+u*action
            if self.command_rate_limit:
                limits=.02*self.command_rate_limit*np.array([1,1,.5,.5])
                action=np.clip(action,self.command_history[-1]-limits,self.command_history[-1]+limits)
            pose=np.eye(4);pose[:3,:3]=rotation(np.radians([self.side*action[0],action[1],0]));pose[:3,3]=[action[2]/1000,self.side*action[3]/1000,.177]
            try:self.checked_ik(pose)
            except ValueError:
                lo=0.;hi=1.;safe=np.eye(4);safe[2,3]=.177
                for _ in range(8):
                    scale=(lo+hi)/2;trial=np.eye(4);trial[:3,:3]=rotation(np.radians([self.side*action[0],action[1],0])*scale);trial[:3,3]=[scale*action[2]/1000,scale*self.side*action[3]/1000,.177]
                    try:self.checked_ik(trial)
                    except ValueError:hi=scale
                    else:lo=scale;safe=trial
                pose=safe;action=action*lo;self.scale=lo;self.projected_frames+=1
            stage='identification' if self.excitation else 'linear edge hold'
        else:
            angles=np.degrees(euler(pose));action=np.array([self.side*angles[0],angles[1],1000*pose[0,3],self.side*1000*pose[1,3]])
            modified=False
            if self.f.energy_pump and t>=1.2:
                angle=math.radians(abs(self.sensed[0]));velocity=math.radians(self.rate[0])
                energy=.5*self.f.energy_inertia_kg_m2*velocity**2+1.475*9.81*(.089*(math.cos(angle)-1)+.056*math.sin(angle))
                desired=self.f.energy_target_j-(.025 if self.side*self.rate[0]<0 else 0)
                action[0]=float(np.clip(self.side*self.rate[0]*self.f.energy_rate_gain*np.clip((desired-energy)/.08,-1,1),-40,40));action[3]=.6*action[0]
                modified=True
            if self.linear is not None and self.f.prebalance_pitch_at_deg:
                if self.pitch_trigger is None and t>.2 and abs(self.sensed[0])>=self.f.prebalance_pitch_at_deg:self.pitch_trigger=t
                if self.pitch_trigger is not None:
                    selected=[1,3,4,7,9,10,13,14,17,18];error=np.zeros(20);error[selected]=(self.state-self.xeq)[selected]
                    pitch_action=np.clip(self.ueq-self.f.prebalance_pitch_gain*self.K@error,[-40,-40,-25,-25],[40,40,25,25])
                    u=blend((t-self.pitch_trigger)/.2);action[[1,2]]=(1-u)*action[[1,2]]+u*pitch_action[[1,2]]
                    modified=True
            if modified:
                candidate=np.eye(4);candidate[:3,:3]=rotation(np.radians([self.side*action[0],action[1],0]));candidate[:3,3]=[action[2]/1000,self.side*action[3]/1000,.177]
                try:self.checked_ik(candidate);pose=candidate
                except ValueError:
                    # Keep the checked launch pose if the combined proposal is unreachable.
                    action=np.array([self.side*angles[0],angles[1],1000*pose[0,3],self.side*1000*pose[1,3]])
        self.current_action=action.copy();self.command_history.append(action.copy())
        return pose,stage,sensed

    def diagnostics(self):
        return dict(**super().diagnostics(),edge_observation=self.observed.tolist(),edge_state=self.state.tolist(),edge_action=self.current_action.tolist())
