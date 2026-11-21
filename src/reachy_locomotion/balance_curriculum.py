"""Evaluation of physical impacts and a controller-driven return to flat."""
import math
import numpy as np

from .geometry import euler,rotation
from .primitives import blend


def edge_ok(row,axis,side,target):
    if axis=='pitch':
        forces=row['foot_front_back_normal_forces_n_back_front'];loaded=int(side==1)
        support=row['foreaft_edge_support'] and forces[loaded]>.5 and forces[1-loaded]<.1
        cross=row['roll_deg']
    else:
        support=row['edge_support'];cross=row['pitch_deg']
    return bool(support and abs(side*row[axis+'_deg']-target)<=2 and abs(row[axis+'_rate_deg_s'])<=5 and abs(cross)<=5)


def impact_metrics(result,axis,side,target):
    rows=result['trace'];events=[]
    for event in result.get('ball_disturbances',[]):
        t=event['first_robot_contact_s'];pre=[];after=[];recovered=None;start=None
        if t is not None:
            pre=[x for x in rows if t-1<=x['time_s']<=t-.04]
            after=[x for x in rows if t<=x['time_s']<=t+3]
            for x in after:
                if edge_ok(x,axis,side,target):
                    if start is None:start=x['time_s']
                    if x['time_s']-start>=.5-1e-8:recovered=start-t;break
                else:start=None
        pre_pass=len(pre)>=30 and all(edge_ok(x,axis,side,target) for x in pre)
        baseline=float(np.mean([x[axis+'_deg'] for x in pre])) if pre else None
        events.append(dict(index=event['index'],contacted_robot=event['contacted_robot'],rebounded=event['rebounded'],
                           balanced_before_impact=pre_pass,recovery_s=recovered,
                           peak_angle_departure_deg=max((abs(side*x[axis+'_deg']-target) for x in after),default=None),
                           peak_impact_angle_change_deg=max((abs(x[axis+'_deg']-baseline) for x in after),default=None) if baseline is not None else None,
                           normal_impulse_ns=event['normal_impulse_ns'],
                           pass_impact=bool(result['completed'] and pre_pass and event['rebounded'] and recovered is not None)))
    return dict(events=events,all_impacts_pass=bool(events) and all(x['pass_impact'] for x in events))


class BalanceReturn:
    """Wrap a controller; return after a continuous hold sensed by that policy.

Return timing is based on its delayed/noisy attitude and rate, not contacts
or truth-state diagnostics. The target remains checked by the original IK.
"""
    def __init__(self,controller,hold_s=5.,roll_deg=-20.,y_mm=15.,duration_s=1.,neutral_at_deg=18.,mode='pose',lower_to_deg=15.,emergency_roll_deg=0.,emergency_rate_deg_s=0.):
        if not 1<=hold_s<=30 or abs(roll_deg)>40 or abs(y_mm)>25 or not .2<=duration_s<=10 or not 3<=neutral_at_deg<=25 or mode not in ('pose','target_ramp') or not 5<=lower_to_deg<=30:
            raise ValueError('Invalid balance return')
        self.controller=controller;self.hold_s=hold_s;self.return_roll=roll_deg;self.return_y=y_mm
        self.duration=duration_s;self.neutral_at=neutral_at_deg;self.hold_start=None;self.return_start=None;self.neutral_start=None
        self.origin=None;self.neutral_origin=None;self.finished=False
        self.mode=mode;self.lower_to=lower_to_deg
        if not 0<=emergency_roll_deg<=45 or not 0<=emergency_rate_deg_s<=60:raise ValueError('Invalid emergency threshold')
        self.emergency_roll=emergency_roll_deg;self.emergency_rate=emergency_rate_deg_s;self.armed=False;self.emergency_triggered=False
        if mode=='target_ramp':
            c=controller;H=np.zeros((20,8));H[2:6,:4]=np.eye(4);H[12:16,4:]=np.eye(4);H[16:20,4:]=np.eye(4)
            U=np.c_[np.zeros((4,4)),np.eye(4)];offset=np.zeros(20);offset[0]=1
            slopes=np.linalg.lstsq((np.eye(20)-c.A)@H-c.B@U,-(np.eye(20)-c.A)@offset,rcond=None)[0]
            self.x_slope=H@slopes+offset;self.u_slope=U@slopes;self.original_xeq=c.xeq.copy();self.original_ueq=c.ueq.copy()

    def __getattr__(self,name):return getattr(self.controller,name)

    def target(self,t,roll_deg):
        if self.mode=='target_ramp' and self.return_start is not None:
            c=self.controller;delta=(self.lower_to-c.f.target_roll_deg)*blend((t-self.return_start)/self.duration)
            c.xeq=self.original_xeq+delta*self.x_slope;c.ueq=self.original_ueq+delta*self.u_slope
        pose,stage,sensed=self.controller.target(t,roll_deg)
        c=self.controller
        # This wrapper lives in the controller's virtual roll frame. For
        # fore-aft use it inside ForeAftController, before command conversion.
        signed=c.side*c.sensed[0]
        stable=abs(signed-c.f.target_roll_deg)<=2 and abs(c.rate[0])<=5 and abs(c.sensed[1])<=5
        if self.return_start is None:
            if stable:
                if self.hold_start is None:self.hold_start=t
                if t-self.hold_start>=.5:self.armed=True
                if t-self.hold_start>=self.hold_s:self.return_start=t;self.origin=pose.copy()
            else:self.hold_start=None
            if self.armed and ((self.emergency_roll and signed>=self.emergency_roll) or (self.emergency_rate and abs(c.rate[0])>=self.emergency_rate)):
                self.return_start=t;self.origin=pose.copy();self.emergency_triggered=True
        if self.return_start is not None:
            if self.mode=='pose':
                target=np.eye(4);target[:3,:3]=rotation(np.radians([c.side*self.return_roll,0,0]));target[:3,3]=[0,c.side*self.return_y/1000,.177]
                u=blend((t-self.return_start)/self.duration)
                pose=np.eye(4);pose[:3,:3]=rotation((1-u)*euler(self.origin)+u*euler(target));pose[:3,3]=(1-u)*self.origin[:3,3]+u*target[:3,3]
            stage='controlled return'
            if self.neutral_start is None and signed<self.neutral_at:self.neutral_start=t;self.neutral_origin=pose.copy()
            if self.neutral_start is not None:
                u=blend((t-self.neutral_start)/.6);pose[:3,:3]=rotation((1-u)*euler(self.neutral_origin));pose[:3,3]=(1-u)*self.neutral_origin[:3,3]+u*np.array([0,0,.177])
                stage='return settle';self.finished=t-self.neutral_start>2
            # Record the actual issued target for subsequent causal estimation.
            a=np.degrees(euler(pose));action=np.array([c.side*a[0],a[1],1000*pose[0,3],c.side*1000*pose[1,3]])
            c.current_action=action.copy();c.command_history[-1]=action.copy()
        return pose,stage,sensed

    def diagnostics(self):
        return dict(**self.controller.diagnostics(),return_started_s=self.return_start,return_neutral_started_s=self.neutral_start,emergency_return=self.emergency_triggered)
