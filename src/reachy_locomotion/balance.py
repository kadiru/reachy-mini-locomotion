"""Delayed roll/rate feedback for the simulation-only edge curriculum."""
import math
from collections import deque
from dataclasses import asdict, dataclass, replace

import numpy as np

from .geometry import euler, rotation
from .primitives import Primitive, PrimitiveController, blend, stroke


@dataclass
class BalanceFeedback:
    prediction_s: float = .08
    trigger_deg: float = 10.
    roll_kp: float = 0.
    roll_kd_s: float = 0.
    y_per_roll_mm_deg: float = 0.
    correction_limit_deg: float = 18.
    rate_filter_s: float = .04
    activation_s: float = .08
    sensor_delay_frames: int = 1
    sensor_noise_deg: float = 0.
    sensor_seed: int = 0
    yaw_delay_after_catch_s: float = .15
    state_dependent_yaw: bool = False
    yaw_return_roll_deg: float = 3.
    workspace_projection: bool = False
    catch_direction: int = 0
    yaw_direction: int = 0
    yaw_prediction_s: float = 0.
    y_roll_kp_mm_deg: float = 0.
    y_roll_kd_mm_s_deg: float = 0.
    return_x_after_catch_s: float = -1.
    return_x_duration_s: float = .6
    yaw_before_catch: bool = False
    workspace_joint_margin_rad: float = 0.
    measured_yaw_brake: bool = False
    yaw_brake_at_deg: float = 20.
    yaw_brake_prediction_s: float = .03
    yaw_align_with_roll: bool = False
    emergency_roll_deg: float = 0.
    emergency_prediction_s: float = .04
    emergency_blend_s: float = .12

    def __post_init__(self):
        if not all(math.isfinite(x) for x in asdict(self).values()):
            raise ValueError('Nonfinite balance parameter')
        if not 0<=self.prediction_s<=.3 or not 2<=self.trigger_deg<=30:
            raise ValueError('Invalid prediction/trigger')
        if abs(self.roll_kp)>4 or abs(self.roll_kd_s)>.6 or abs(self.y_per_roll_mm_deg)>2:
            raise ValueError('Invalid feedback gains')
        if not 0<self.correction_limit_deg<=30 or not .02<=self.rate_filter_s<=.2 or not .02<=self.activation_s<=.5:
            raise ValueError('Invalid filter or correction limit')
        if type(self.sensor_delay_frames) is not int or not 0<=self.sensor_delay_frames<=5 or not 0<=self.sensor_noise_deg<=1:
            raise ValueError('Invalid simulated sensor')
        if not 0<=self.yaw_delay_after_catch_s<=1:
            raise ValueError('Invalid yaw delay')
        if type(self.state_dependent_yaw) is not bool or type(self.workspace_projection) is not bool or not 1<=self.yaw_return_roll_deg<=8:
            raise ValueError('Invalid yaw/projection option')
        if self.catch_direction not in (-1,0,1) or self.yaw_direction not in (-1,0,1) or not 0<=self.yaw_prediction_s<=.2:
            raise ValueError('Invalid signed phase trigger')
        if abs(self.y_roll_kp_mm_deg)>3 or abs(self.y_roll_kd_mm_s_deg)>.4:
            raise ValueError('Invalid lateral balance gains')
        if not -1<=self.return_x_after_catch_s<=4 or not .2<=self.return_x_duration_s<=1.5:
            raise ValueError('Invalid forward-offset reset timing')
        if type(self.yaw_before_catch) is not bool:raise ValueError('Invalid yaw phase option')
        if not 0<=self.workspace_joint_margin_rad<=.15:raise ValueError('Invalid command joint margin')
        if type(self.measured_yaw_brake) is not bool or not 5<=self.yaw_brake_at_deg<=40 or not 0<=self.yaw_brake_prediction_s<=.1:
            raise ValueError('Invalid measured yaw brake')
        if type(self.yaw_align_with_roll) is not bool:raise ValueError('Invalid yaw direction option')
        if self.emergency_roll_deg and not 15<=self.emergency_roll_deg<=30:raise ValueError('Invalid recovery override')
        if not 0<=self.emergency_prediction_s<=.15 or not .04<=self.emergency_blend_s<=.3:
            raise ValueError('Invalid recovery override timing')


class BalanceController:
    def __init__(self, recipe, feedback,checked_ik=None):
        self.p=recipe if isinstance(recipe,Primitive) else Primitive(**recipe)
        self.f=feedback if isinstance(feedback,BalanceFeedback) else BalanceFeedback(**feedback)
        # Trigger the existing bounded response ourselves using predicted roll.
        self.base=PrimitiveController(replace(self.p,catch_at_deg=0,yaw_at_roll_deg=60,yaw_deg=0 if self.f.state_dependent_yaw else self.p.yaw_deg))
        self.history=deque([0.]*(self.f.sensor_delay_frames+1),maxlen=self.f.sensor_delay_frames+1)
        self.rng=np.random.default_rng(self.f.sensor_seed)
        self.previous_roll=0.;self.rate=0.;self.sensed=0.;self.predicted=0.;self.correction=0.
        self.catch_trigger=None;self.yaw_trigger=None
        self.yaw_return_start=None;self.checked_ik=checked_ik;self.projected_frames=0;self.projection_scale=1.
        self.head_history=deque([0.,0.],maxlen=2);self.head_yaw=0.;self.head_rate=0.;self.brake_target=None
        self.yaw_amplitude=self.p.yaw_deg
        self.emergency_trigger=None;self.emergency_origin=None

    def observe_head(self,t,yaw_deg):
        if not self.f.measured_yaw_brake:return
        self.head_history.append(yaw_deg)
        sensed=self.head_history[0];raw=(sensed-self.head_yaw)/.02 if t else 0.
        self.head_rate+=(1-math.exp(-.02/.04))*(raw-self.head_rate)
        self.head_yaw=sensed

    def projected_ik(self,pose):
        joints=self.checked_ik(pose)
        if self.f.workspace_joint_margin_rad:
            limits=self.checked_ik.__self__.command_limits
            if min(float(np.min(joints-limits[:,0])),float(np.min(limits[:,1]-joints)))<self.f.workspace_joint_margin_rad:
                raise ValueError('Reserve inward command margin for dynamic overshoot')
        return joints

    def target(self,t,roll_deg):
        f=self.f
        self.history.append(roll_deg+float(self.rng.normal(0,f.sensor_noise_deg)))
        self.sensed=float(self.history[0])
        raw_rate=(self.sensed-self.previous_roll)/.02 if t else 0.
        self.previous_roll=self.sensed
        self.rate+=(1-math.exp(-.02/f.rate_filter_s))*(raw_rate-self.rate)
        self.predicted=self.sensed+f.prediction_s*self.rate
        side=1 if self.p.roll_deg>=0 else -1
        risk=abs(self.predicted) if not f.catch_direction else f.catch_direction*side*self.predicted
        if self.base.catch_trigger is None and t>=.2+self.p.preload_s and risk>=f.trigger_deg:
            self.base.catch_trigger=t
        if self.p.yaw_deg and (self.base.catch_trigger is not None or f.yaw_before_catch) and self.base.yaw_trigger is None:
            predicted_yaw_roll=self.sensed+f.yaw_prediction_s*self.rate
            yaw_phase=abs(predicted_yaw_roll) if not f.yaw_direction else f.yaw_direction*side*predicted_yaw_roll
            earliest=.2+self.p.preload_s if f.yaw_before_catch else self.base.catch_trigger+f.yaw_delay_after_catch_s
            if t>=earliest and yaw_phase>=self.p.yaw_at_roll_deg:
                self.base.yaw_trigger=t
                if f.yaw_align_with_roll:self.yaw_amplitude=math.copysign(abs(self.p.yaw_deg),predicted_yaw_roll)
        pose,stage,_=self.base.target(t,0.)
        self.catch_trigger=self.base.catch_trigger;self.yaw_trigger=self.base.yaw_trigger
        self.correction=0.
        if self.catch_trigger is not None:
            active=blend((t-self.catch_trigger)/f.activation_s)
            self.correction=float(np.clip(f.roll_kp*self.sensed+f.roll_kd_s*self.rate,-f.correction_limit_deg,f.correction_limit_deg))*active
            angles=euler(pose)
            angles[0]=math.radians(float(np.clip(math.degrees(angles[0])+self.correction,-40,40)))
            pose[:3,:3]=rotation(angles)
            lateral=f.y_per_roll_mm_deg*self.correction+active*(f.y_roll_kp_mm_deg*self.sensed+f.y_roll_kd_mm_s_deg*self.rate)
            pose[1,3]=float(np.clip(pose[1,3]+lateral/1000,-.025,.025))
            stage='feedback '+stage
        if f.state_dependent_yaw and self.yaw_trigger is not None:
            elapsed=t-self.yaw_trigger
            if self.yaw_return_start is None and elapsed>=self.p.yaw_s+self.p.yaw_hold_s:
                if (abs(self.sensed)<f.yaw_return_roll_deg and abs(self.rate)<30) or elapsed>=1.5:
                    self.yaw_return_start=t
            yaw=self.yaw_amplitude*stroke(elapsed/self.p.yaw_s,self.p.yaw_brake_fraction)
            if f.measured_yaw_brake:
                sign=1 if self.yaw_amplitude>0 else -1
                if self.brake_target is None and sign*self.head_yaw>=f.yaw_brake_at_deg and abs(self.sensed)>=5:
                    self.brake_target=float(np.clip(self.head_yaw-f.yaw_brake_prediction_s*self.head_rate,-60,60))
                yaw=self.yaw_amplitude if self.brake_target is None else self.brake_target
            if self.yaw_return_start is not None:
                held=self.yaw_amplitude if self.brake_target is None else self.brake_target
                yaw=held*(1-blend((t-self.yaw_return_start)/self.p.yaw_return_s))
            angles=euler(pose);angles[2]=math.radians(yaw);pose[:3,:3]=rotation(angles)
        self.projection_scale=1.
        if f.return_x_after_catch_s>=0 and self.catch_trigger is not None:
            pose[0,3]*=1-blend((t-self.catch_trigger-f.return_x_after_catch_s)/f.return_x_duration_s)
        # A latched neutral recovery response, driven only by delayed roll/rate.
        # The motor profile and the normal IK projection still bound its motion.
        if f.emergency_roll_deg:
            risk=self.sensed+f.emergency_prediction_s*self.rate
            if self.emergency_trigger is None and abs(risk)>=f.emergency_roll_deg and self.sensed*self.rate>0:
                self.emergency_trigger=t;self.emergency_origin=pose.copy()
            if self.emergency_trigger is not None:
                correction=float(np.clip(1.5*self.sensed-.12*self.rate,-18,18))
                u=blend((t-self.emergency_trigger)/f.emergency_blend_s)
                angles=(1-u)*euler(self.emergency_origin)+u*np.array([math.radians(correction),0.,0.])
                pose[:3,:3]=rotation(angles)
                pose[:3,3]=(1-u)*self.emergency_origin[:3,3]+u*np.array([0.,.0006*correction,.177])
                stage='emergency recovery'
        if f.workspace_projection:
            if self.checked_ik is None:raise ValueError('Projection requires the same checked IK as the simulator')
            try:self.projected_ik(pose)
            except ValueError:
                angles=euler(pose);offset=pose[:3,3]-np.array([0.,0.,.177]);lo=0.;hi=1.
                safe=np.eye(4);safe[2,3]=.177
                self.projected_ik(safe)
                for _ in range(7):
                    scale=(lo+hi)/2;trial=np.eye(4);trial[:3,:3]=rotation(angles*scale);trial[:3,3]=np.array([0.,0.,.177])+offset*scale
                    try:self.projected_ik(trial)
                    except ValueError:hi=scale
                    else:lo=scale;safe=trial
                pose=safe;self.projection_scale=lo;self.projected_frames+=1
        return pose,stage,self.sensed

    def diagnostics(self):
        return dict(sensed_roll_rate_deg_s=self.rate,predicted_roll_deg=self.predicted,
                    feedback_roll_correction_deg=self.correction,workspace_projection_scale=self.projection_scale,
                    sensed_head_yaw_deg=self.head_yaw,sensed_head_yaw_rate_deg_s=self.head_rate,brake_target_deg=self.brake_target,
                    selected_yaw_target_deg=self.yaw_amplitude,emergency_recovery_trigger_s=self.emergency_trigger)


def mirrored(recipe,side):
    if side not in (-1,1):raise ValueError('Side must be -1 or +1')
    values=asdict(recipe) if isinstance(recipe,Primitive) else dict(recipe)
    for key in ['preload_roll_deg','roll_deg','preload_y_mm','y_mm','catch_roll_deg','catch_y_mm','yaw_deg']:
        values[key]*=side
    return Primitive(**values)
