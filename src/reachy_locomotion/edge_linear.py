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
        self.observer=None
        self.gyro=None;self.gyro_noise_std=None;self.gyro_filter_s=0.;self.filtered_gyro=None
        self.gyro_rng=np.random.default_rng(self.f.sensor_seed+4001)
        self.attitude_correction_s=0.;self.filtered_attitude=None
        self.gyro_mode='lowpass';self.previous_gyro=None
        self.previous_raw_attitude=None
        self.attitude_rate_weight=0.
        self.hold_transition_s=0.;self.capture_gain=None
        self.gyro_interval_average=False
        self.capture_attitude_correction_s=None
        self.velocity_command_rate_gain=0.
        self.disturbance_tracking_limit=0.
        self.stable_samples=0
        self.recovery_armed=False
        self.recovery_until=-1.
        self.recovery_frames=0
        if linear is not None:
            self.A=np.array(linear['A']);self.B=np.array(linear['B']);self.c=np.array(linear['c'])
            self.K=np.array(linear['gain']);self.xeq=np.array(linear['x_equilibrium']);self.ueq=np.array(linear['u_equilibrium'])
            for value,shape in [(self.A,(20,20)),(self.B,(20,4)),(self.c,(20,)),(self.K,(4,20)),(self.xeq,(20,)),(self.ueq,(4,))]:
                if value.shape!=shape or not np.isfinite(value).all():raise ValueError('Invalid learned edge model')

    def observe_head_state(self,t,pose):
        self.head=np.r_[np.degrees(euler(pose)[:2]),1000*pose[:2,3]]

    def configure_observer(self, parameters):
        if parameters is not None:
            if self.linear is None:raise ValueError('Observer requires an identified model')
            from .edge_observer import EdgeObserver
            self.observer=EdgeObserver(self.A,self.B,self.c,**parameters)

    def configure_sensors(self, parameters):
        if parameters is None:return
        self.gyro_noise_std=float(parameters.get('gyro_std_deg_s',.1));self.gyro_filter_s=float(parameters.get('gyro_filter_s',0.))
        self.attitude_correction_s=float(parameters.get('attitude_correction_s',0.))
        self.gyro_mode=parameters.get('gyro_mode','lowpass')
        self.gyro_interval_average=bool(parameters.get('gyro_interval_average',False))
        self.attitude_rate_weight=float(parameters.get('attitude_rate_weight',0.))
        if not 0 <= self.attitude_rate_weight <= 1:
            raise ValueError('Attitude rate weight must be between zero and one')
        self.capture_attitude_correction_s=parameters.get('capture_attitude_correction_s')
        if self.capture_attitude_correction_s is not None and not 0<self.capture_attitude_correction_s<=2:raise ValueError('Invalid capture attitude correction')
        if self.gyro_mode not in ('lowpass','trapezoid'):raise ValueError('Invalid gyro integration')
        if not 0<=self.gyro_noise_std<=10 or not 0<=self.gyro_filter_s<=.2 or not 0<=self.attitude_correction_s<=2:raise ValueError('Invalid simulated gyro')

    def configure_phase_tuning(self, parameters):
        if parameters is None:return
        self.hold_transition_s=float(parameters.get('hold_transition_s',.8));self.capture_gain=float(parameters.get('capture_gain_scale',1.2))
        if not 0<=self.hold_transition_s<=3 or not 0<=self.capture_gain<=2:raise ValueError('Invalid capture transition')

    def observe_angular_velocity(self,t,body_angular_velocity_rad_s):
        self.gyro=np.asarray(body_angular_velocity_rad_s).copy()

    def target(self,t,roll_deg):
        # Signed roll and head roll/y are mirrored; pitch/x remain unchanged.
        reference=self.linear.get('reference_roll_deg',32.1) if self.linear is not None else self.f.target_roll_deg
        h=self.head*np.array([self.side,1,1,self.side]);position=np.r_[self.side*roll_deg-reference,self.pitch,h]
        noise=self.rng.normal(0,self.f.sensor_noise_deg,2);position[:2]+=noise
        self._sensor_noise_sample=noise*np.array([self.side,1])
        velocity=np.zeros(6) if self.previous_observation is None else (position-self.previous_observation)/.02
        if self.gyro_noise_std is not None:
            if self.gyro is None:raise ValueError('Configured gyro requires a simulated IMU sample')
            measured=self.gyro+np.radians(self.gyro_rng.normal(0,self.gyro_noise_std,3))
            if self.gyro_mode=='trapezoid':
                self.filtered_gyro=measured.copy() if self.previous_gyro is None else .5*(measured+self.previous_gyro)
            elif self.filtered_gyro is None:self.filtered_gyro=measured
            else:
                alpha=1-math.exp(-.02/self.gyro_filter_s) if self.gyro_filter_s else 1.
                self.filtered_gyro+=alpha*(measured-self.filtered_gyro)
            self.previous_gyro=measured.copy()
            p,q,r=self.filtered_gyro;phi=math.radians(self.side*(position[0]+reference));theta=math.radians(position[1])
            velocity[:2]=np.degrees([p+math.sin(phi)*math.tan(theta)*q+math.cos(phi)*math.tan(theta)*r,
                                     math.cos(phi)*q-math.sin(phi)*r])*[self.side,1]
            if self.attitude_rate_weight:
                raw_attitude = position[:2].copy()
                if self.previous_raw_attitude is not None:
                    interval_rate = (raw_attitude-self.previous_raw_attitude)/.02
                    velocity[:2] += self.attitude_rate_weight*(interval_rate-velocity[:2])
                self.previous_raw_attitude = raw_attitude
            if self.attitude_correction_s:
                if self.filtered_attitude is None:self.filtered_attitude=position[:2].copy()
                else:
                    predicted=self.filtered_attitude+.02*velocity[:2]
                    tau=self.attitude_correction_s
                    if self.capture_attitude_correction_s is not None and (self.catch_trigger is None or t-self.catch_trigger<max(1.,self.hold_transition_s)):
                        tau=self.capture_attitude_correction_s
                    self.filtered_attitude=predicted+(1-math.exp(-.02/tau))*(position[:2]-predicted)
                position[:2]=self.filtered_attitude
                self._sensor_noise_sample=(position[:2]-[self.side*roll_deg-reference,self.pitch])*[self.side,1]
        self.previous_observation=position.copy();self.observed=np.r_[position,velocity]
        if self.observer is not None:
            self.observed=self.observer.update(position,self.command_history[-1],list(self.command_history)[-2:])
        self.sensor_history.append(self.observed.copy())
        if self.disturbance_tracking_limit and self.catch_trigger is not None:
            delayed = self.sensor_history[0]
            quiet = abs(delayed[0]) < 1.2 and abs(delayed[1]) < 4 and np.linalg.norm(delayed[6:8]) < 5
            self.stable_samples = self.stable_samples+1 if quiet else 0
            self.recovery_armed |= self.stable_samples >= 25
            if self.recovery_armed and (abs(delayed[0]) > 1.8 or np.linalg.norm(delayed[6:8]) > 9):
                self.recovery_until = t+.4
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
                gain=self.control_gain
                if self.capture_gain is not None:
                    u=blend((t-self.catch_trigger-self.hold_transition_s)/.2);gain=(1-u)*self.capture_gain+u*gain
                action=self.ueq-gain*self.K@(self.state-self.xeq)
            else:action=np.zeros(4)
            recovering = t <= self.recovery_until
            self.recovery_frames += int(recovering)
            tracking_limit = self.disturbance_tracking_limit if recovering else self.tracking_error_limit
            if tracking_limit:
                limits=tracking_limit*np.array([1,1,.5,.5])
                action=np.clip(action,self.state[2:6]-limits,self.state[2:6]+limits)
            action=np.clip(action,[-40,-40,-25,-25],[40,40,25,25])
            if not self.hold_from_start:
                a=np.degrees(euler(self.capture_pose));origin=np.array([self.side*a[0],a[1],1000*self.capture_pose[0,3],self.side*1000*self.capture_pose[1,3]])
                u=blend((t-self.catch_trigger)/self.f.capture_blend_s);action=(1-u)*origin+u*action
            if self.command_rate_limit and t-self.catch_trigger>=self.hold_transition_s:
                rate_limit=min(800.,self.command_rate_limit+self.velocity_command_rate_gain*float(np.linalg.norm(self.state[6:8])))
                limits=.02*rate_limit*np.array([1,1,.5,.5])
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
