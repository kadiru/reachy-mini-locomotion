"""Simulation-only sustained edge balance with delayed attitude feedback.

Natural head-driven launch; no base state writes, contact/COM observations,
external forces, waist rotation commands or changes to physical motor bounds.
"""
import math
from collections import deque
from dataclasses import asdict, dataclass, replace

import numpy as np

from .geometry import euler, rotation
from .primitives import Primitive, PrimitiveController, blend
from .balance import mirrored


@dataclass
class EdgeFeedback:
    launch_side: int = -1
    preserve_launch_catch: bool = True
    launch_return_x_after_s: float = -1.
    launch_return_x_duration_s: float = .4
    prebalance_pitch_at_deg: float = 0.
    prebalance_pitch_gain: float = 1.
    launch_interpolate_catch: bool = False
    capture_interpolate: bool = False
    energy_pump: bool = False
    energy_target_j: float = .23
    energy_rate_gain: float = .4
    energy_inertia_kg_m2: float = .0278
    target_roll_deg: float = 30.
    trigger_deg: float = 20.
    prediction_s: float = .08
    capture_blend_s: float = .08
    roll_bias_deg: float = 0.
    y_bias_mm: float = 0.
    roll_kp: float = -2.
    roll_kd_s: float = .15
    y_kp_mm_deg: float = 1.
    y_kd_mm_s_deg: float = -.1
    roll_ki: float = 0.
    pitch_bias_deg: float = 0.
    x_bias_mm: float = 0.
    pitch_kp: float = 0.
    pitch_kd_s: float = 0.
    x_kp_mm_deg: float = 0.
    x_kd_mm_s_deg: float = 0.
    z_mm: float = 0.
    rate_filter_s: float = .04
    sensor_delay_frames: int = 1
    sensor_noise_deg: float = 0.
    sensor_seed: int = 0

    def __post_init__(self):
        if self.launch_side not in (-1,1):raise ValueError('Invalid launch side')
        if type(self.preserve_launch_catch) is not bool:raise ValueError('Invalid launch option')
        if type(self.launch_interpolate_catch) is not bool:raise ValueError('Invalid launch interpolation')
        if type(self.capture_interpolate) is not bool:raise ValueError('Invalid capture interpolation')
        if type(self.energy_pump) is not bool or not .1<=self.energy_target_j<=.4 or not .05<=self.energy_rate_gain<=2 or not .01<=self.energy_inertia_kg_m2<=.05:raise ValueError('Invalid energy pump')
        if not -1<=self.launch_return_x_after_s<=1 or not .1<=self.launch_return_x_duration_s<=1:raise ValueError('Invalid launch x return')
        if not 0<=self.prebalance_pitch_at_deg<=25 or not .2<=self.prebalance_pitch_gain<=2:raise ValueError('Invalid prebalance pitch feedback')
        if not all(math.isfinite(x) for x in asdict(self).values()):raise ValueError('Nonfinite edge feedback')
        if not 20<=self.target_roll_deg<=33 or not 5<=self.trigger_deg<=33:raise ValueError('Invalid balance target/trigger')
        if not 0<=self.prediction_s<=.25 or not .02<=self.capture_blend_s<=.5:raise ValueError('Invalid capture timing')
        if max(abs(self.roll_bias_deg),abs(self.pitch_bias_deg))>40:raise ValueError('Invalid angle bias')
        if max(abs(self.y_bias_mm),abs(self.x_bias_mm),abs(self.z_mm))>25:raise ValueError('Invalid position bias')
        if max(abs(self.roll_kp),abs(self.pitch_kp),abs(self.y_kp_mm_deg),abs(self.x_kp_mm_deg))>20:raise ValueError('Invalid proportional gain')
        if max(abs(self.roll_kd_s),abs(self.pitch_kd_s),abs(self.y_kd_mm_s_deg),abs(self.x_kd_mm_s_deg))>3 or abs(self.roll_ki)>10:raise ValueError('Invalid dynamic gain')
        if not .02<=self.rate_filter_s<=.2:raise ValueError('Invalid rate filter')
        if type(self.sensor_delay_frames) is not int or not 0<=self.sensor_delay_frames<=5 or not 0<=self.sensor_noise_deg<=1:raise ValueError('Invalid sensor')


class EdgeBalanceController:
    def __init__(self, recipe, feedback, side, checked_ik):
        self.f=feedback if isinstance(feedback,EdgeFeedback) else EdgeFeedback(**feedback)
        self.p=recipe if isinstance(recipe,Primitive) else Primitive(**recipe)
        if side not in (-1,1):raise ValueError('Invalid side')
        self.side=side;self.checked_ik=checked_ik
        self.launch=PrimitiveController(mirrored(replace(self.p,catch_at_deg=self.p.catch_at_deg if self.f.preserve_launch_catch else 0,yaw_deg=0),side*self.f.launch_side))
        self.history=deque([np.zeros(2) for _ in range(self.f.sensor_delay_frames+1)],maxlen=self.f.sensor_delay_frames+1)
        self.rng=np.random.default_rng(self.f.sensor_seed);self.sensed=np.zeros(2);self.rate=np.zeros(2);self.pitch=0.
        self.catch_trigger=None;self.yaw_trigger=None;self.capture_pose=None;self.integral=0.
        self.projected_frames=0;self.scale=1.;self.predicted=0.
        self.stop_when_flat=False;self.finished=False
        self.hold_from_start=False

    def observe_attitude(self,t,pitch_deg):self.pitch=pitch_deg

    def target(self,t,roll_deg):
        if self.hold_from_start and t==0:
            # The artificial reset assumes prior stationary sensing at that
            # posture. Avoid inventing a 32-degree sensor jump at t=20 ms.
            initial=np.array([roll_deg,self.pitch]);self.sensed=initial.copy()
            for i in range(len(self.history)):self.history[i]=initial.copy()
        f=self.f
        noise=self.__dict__.pop('_sensor_noise_sample',None)
        if noise is None:noise=self.rng.normal(0,f.sensor_noise_deg,2)
        self.history.append(np.array([roll_deg,self.pitch])+noise)
        sensed=self.history[0];raw=(sensed-self.sensed)/.02 if t else np.zeros(2)
        self.rate+=(1-math.exp(-.02/f.rate_filter_s))*(raw-self.rate);self.sensed=sensed.copy()
        roll,rate=self.side*self.sensed[0],self.side*self.rate[0]
        self.predicted=roll+f.prediction_s*rate
        if f.launch_interpolate_catch and self.launch.catch_trigger is None and self.p.catch_at_deg and t>=.2+self.p.preload_s and abs(self.sensed[0])>=self.p.catch_at_deg:
            age=float(np.clip((abs(self.sensed[0])-self.p.catch_at_deg)/max(abs(self.rate[0]),1),0,.02))
            self.launch.catch_trigger=t-age
        pose,stage,_=self.launch.target(t,roll_deg)
        if self.stop_when_flat and self.launch.catch_trigger is not None and t>self.launch.catch_trigger+1.5 and abs(roll)<1 and abs(rate)<2:
            self.finished=True
        if f.launch_return_x_after_s>=0 and self.launch.catch_trigger is not None:
            pose[0,3]*=1-blend((t-self.launch.catch_trigger-f.launch_return_x_after_s)/f.launch_return_x_duration_s)
        if self.catch_trigger is None and (self.hold_from_start or (t>=.2+self.p.preload_s and self.predicted>=f.trigger_deg)):
            age=float(np.clip((self.predicted-f.trigger_deg)/max(rate,1),0,.02)) if f.capture_interpolate and not self.hold_from_start else 0.
            self.catch_trigger=t-age;self.capture_pose=pose.copy()
        if self.catch_trigger is not None:
            if self.stop_when_flat and t>self.catch_trigger+1 and abs(roll)<1 and abs(rate)<2:self.finished=True
            error=roll-f.target_roll_deg
            self.integral=float(np.clip(self.integral+.02*error,-10,10))
            head_roll=self.side*(f.roll_bias_deg+f.roll_kp*error+f.roll_kd_s*rate+f.roll_ki*self.integral)
            y=self.side*(f.y_bias_mm+f.y_kp_mm_deg*error+f.y_kd_mm_s_deg*rate)
            head_pitch=f.pitch_bias_deg+f.pitch_kp*self.sensed[1]+f.pitch_kd_s*self.rate[1]
            x=f.x_bias_mm+f.x_kp_mm_deg*self.sensed[1]+f.x_kd_mm_s_deg*self.rate[1]
            a=np.radians(np.clip([head_roll,head_pitch,0],-40,40));xyz=np.array([np.clip(x,-25,25)/1000,np.clip(y,-25,25)/1000,.177+f.z_mm/1000])
            u=blend((t-self.catch_trigger)/f.capture_blend_s)
            pose[:3,:3]=rotation((1-u)*euler(self.capture_pose)+u*a);pose[:3,3]=(1-u)*self.capture_pose[:3,3]+u*xyz
            stage='capture' if u<1 else 'edge balance'
        self.scale=1.
        try:self.checked_ik(pose)
        except ValueError:
            a=euler(pose);offset=pose[:3,3]-[0,0,.177];lo=0.;hi=1.;safe=np.eye(4);safe[2,3]=.177
            self.checked_ik(safe)
            for _ in range(7):
                mid=(lo+hi)/2;trial=np.eye(4);trial[:3,:3]=rotation(a*mid);trial[:3,3]=np.array([0,0,.177])+mid*offset
                try:self.checked_ik(trial)
                except ValueError:hi=mid
                else:lo=mid;safe=trial
            pose=safe;self.scale=lo;self.projected_frames+=1
        return pose,stage,float(self.sensed[0])

    def diagnostics(self):
        return dict(sensed_roll_rate_deg_s=float(self.rate[0]),sensed_pitch_deg=float(self.sensed[1]),
                    sensed_pitch_rate_deg_s=float(self.rate[1]),predicted_signed_roll_deg=float(self.predicted),
                    workspace_projection_scale=self.scale,balance_target_roll_deg=self.side*self.f.target_roll_deg)


def balance_metrics(result,side,target):
    trace=result['trace'];continuous=best=near=0.;strict=[];com_offsets=[];previous_time=None
    for row in trace:
        error=abs(side*row['roll_deg']-target)
        ok=row['edge_support'] and error<=2 and abs(row['roll_rate_deg_s'])<=5 and abs(row['pitch_deg'])<=5
        step=row['time_s']-previous_time if previous_time is not None else 0.
        continuous=continuous+step if ok and strict and strict[-1] else 0.;best=max(best,continuous)
        strict.append(ok);previous_time=row['time_s']
        if row['edge_support']:
            near+=.02*math.exp(-.5*(error/5)**2-.5*(row['roll_rate_deg_s']/35)**2-.5*(row['pitch_deg']/5)**2)
        if row['contact_world_m'] is not None:
            com_offsets.append(1000*float(np.linalg.norm(np.array(row['com_world_m'][:2])-np.array(row['contact_world_m'][:2]))))
        else:com_offsets.append(None)
    tail=trace[-251:];n=len(tail)
    tail_com=[x for x in com_offsets[-251:] if x is not None]
    return dict(longest_strict_hold_s=round(best,5),soft_hold_s=near,
                final_five_seconds_pass=bool(result['completed'] and n==251 and all(strict[-251:])),
                final_roll_deg=trace[-1]['roll_deg'],final_roll_rate_deg_s=trace[-1]['roll_rate_deg_s'],
                tail_roll_range_deg=[min(x['roll_deg'] for x in tail),max(x['roll_deg'] for x in tail)],
                tail_max_roll_rate_deg_s=max(abs(x['roll_rate_deg_s']) for x in tail),
                tail_mean_com_contact_distance_mm=float(np.mean(tail_com)) if tail_com else None,
                target_roll_deg=side*target,strict_definition='One-edge support; roll error <=2 deg; |roll rate| <=5 deg/s; |pitch| <=5 deg')
