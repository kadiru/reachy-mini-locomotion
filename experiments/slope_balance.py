"""Natural flat-foot launches on CAD ramps with gravity vertical and the waist fixed."""
import argparse
import math
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from pathlib import Path

import numpy as np
from reachy_locomotion.geometry import euler, rotation
from reachy_locomotion.primitives import Primitive, PrimitiveSimulator, blend
from reachy_locomotion.release_plane import ReleasePlane
from tilted_release import MOTORS, UPSTREAM, balance_controller, read, write

WORK=Path('experiments/generated/slope-balance')
POLICY=Path('configs/slope-balance.json')
REPORT=Path('experiments/reports/motor-training-slope-balance.json')


@lru_cache(maxsize=4)
def plant(bank,dt,friction):
    cfg=read(MOTORS)
    cfg.update(cad_shell_floor=True,floor_roll_deg=bank,timestep_s=dt)
    cfg['friction_slide_torsion_roll'][0]=friction
    return PrimitiveSimulator(UPSTREAM,cfg)


def action_pose(side,action):
    roll,y,z=action
    pose=np.eye(4)
    pose[:3,:3]=rotation(np.radians([side*roll,0.,0.]))
    pose[:3,3]=[0.,side*y/1000,.177+z/1000]
    return pose


class RampPump:
    """Estimate bank before launch, pump the head, then catch with the CAD regulator."""
    gyro_interval_average=True
    yaw_trigger=None
    def __init__(self,sim,side,params,seed):
        self.sim,self.side,self.params=sim,side,params
        self.rng=np.random.default_rng(seed)
        self.bank_samples=[]
        self.gyro=np.zeros(3)
        self.pitch=0.
        self.head=np.eye(4)
        self.catch_trigger=None
        self.prev_roll=self.prev_rate=0.
        self.selected={}
        self.last_head=None
        self.head_rate=np.zeros(2)
        self.previous_head_rate=np.zeros(2)
        self.history=[]
        self.strength=1.
        self.bank=0.
        self.catch=None
        self.seed=seed

    def observe_attitude(self,t,pitch):
        self.pitch=pitch

    def observe_head_state(self,t,pose):
        self.head=pose
        current=self.side*np.array([np.degrees(euler(pose)[0]),1000*pose[1,3]])
        if self.last_head is not None:
            self.head_rate=(current-self.last_head)/.02
        self.last_head=current

    def observe_angular_velocity(self,t,velocity):
        self.gyro=velocity.copy()

    def target(self,t,roll):
        pose,stage,measured=self.launch_target(t,roll)
        p=self.selected
        gain=p.get('pitch_damping',0.)
        if not gain or t<p.get('stop_s',0.):
            return pose,stage,measured
        phi=math.radians(roll)
        pitch_rate=math.degrees(math.cos(phi)*self.gyro[1]-math.sin(phi)*self.gyro[2])
        head_pitch=math.degrees(euler(self.head)[1])
        head_x=1000*self.head[0,3]
        correction=np.clip(-gain*(3.6*self.pitch+.62*pitch_rate+1.25*head_pitch+.9*head_x),-18.,18.)
        angles=euler(pose)
        for scale in (1.,.7,.4,.2):
            trial=pose.copy()
            trial[:3,:3]=rotation(angles+np.radians([0.,scale*correction,0.]))
            trial[0,3]+=.00055*scale*correction
            try:self.sim.checked_ik(trial)
            except ValueError:continue
            return trial,stage,measured
        return pose,stage,measured

    def launch_target(self,t,roll):
        measured=roll+self.rng.normal(0,.05)
        rate=self.side*np.degrees(self.gyro[0])+self.rng.normal(0,.1)
        if t<.20-1e-8:
            self.bank_samples.append(measured)
        if not self.selected and t>=.20-1e-8:
            self.bank=float(np.median(self.bank_samples))
            if 'bins' in self.params:
                k=min(self.params['bins'],key=lambda v:abs(v['relative_bank']-self.side*self.bank))
                self.selected=k['settings']
            else:
                self.selected=self.params
            self.strength=self.params.get('strength',self.selected.get('strength',1.))
            if 'bank_gain' in self.selected:
                error=self.side*self.bank-self.selected['reference_bank']
                delta=math.copysign(max(0.,abs(error)-.08),error)
                self.strength*=float(np.clip(1.+self.selected['bank_gain']*delta,.75,1.25))
        p=self.selected or self.params
        a=self.side*self.prev_roll
        v=self.prev_rate
        self.prev_roll,self.prev_rate=measured,rate
        h=self.previous_head_rate
        self.previous_head_rate=self.head_rate.copy()
        coast_rate=v+p.get('inertia',0.)*(.15*h[0]-.22*h[1])
        if (self.selected and t>=p.get('unlock',0.) and self.catch_trigger is None
                and a>p.get('min_trigger',16.) and coast_rate<=p.get('max_rate',1e9)
                and a+p['lead']*coast_rate>=p['trigger']):
            self.catch_trigger=t
            c=read('configs/cad-release-catch.json')['sides']['1']['specialists'][3]
            trim=p.get('trim')
            if trim:
                u=trim['action']
                x=[trim['equilibrium']-32.1,trim['head_roll'],trim['head_y'],0.,0.,0.,*u,*u]
                c=dict(c,model=dict(c['model'],xeq=x,ueq=u),neutral_trim=True)
            if p.get('catch_model')=='full' and not trim:
                tune=dict(seed=self.seed,gain=.9,rate_limit=120,pitch_bias=2.,pitch_gain=.7)
                self.catch=balance_controller(self.sim,self.side,tune)
            else:
                self.catch=ReleasePlane(self.side,self.seed,self.sim.checked_ik,c)
                if p.get('warm') and len(self.history)>=3:
                    for cmd in self.history[-3:]:
                        self.catch.commands.append(cmd.copy())
                    self.catch.previous=np.r_[a-32.1,self.last_head-.02*self.head_rate]
        if self.catch:
            self.catch.observe_attitude(t,self.pitch)
            self.catch.observe_head_state(t,self.head)
            self.catch.observe_angular_velocity(t,self.gyro)
            return self.catch.target(t-self.catch_trigger,roll)
        pose=np.eye(4);pose[2,3]=.177
        if not self.selected:
            return pose,'measure ramp',measured
        if t>=p.get('stop_s',1e9):
            trim=p.get('trim')
            u=trim['action'] if trim else [0.,0.]
            self.history.append(np.asarray(u))
            return action_pose(self.side,[*u,0.]),'coast',measured
        elapsed=t-.20
        if 'actions' in p:
            frame=max(0,round(elapsed/.02))
            action=p['actions'][min(frame//p['frames'],len(p['actions'])-1)]
            if frame>=p['frames']*len(p['actions']) and p.get('coast'):
                action=[0.,0.,0.]
            scaled=np.asarray(action)*[self.strength,self.strength,1.]
            for scale in (1.,.92,.8,.65,.4,0.):
                pose=action_pose(self.side,scaled*scale)
                try:
                    self.sim.checked_ik(pose)
                    break
                except ValueError:
                    if not scale:raise
            self.history.append(scaled[:2]*scale)
            return pose,'slope stroke',measured
        if 'durations' in p:
            ends=np.cumsum(p['durations'])
            idx=int(np.searchsorted(ends,elapsed,side='right'))
            start=0. if idx==0 else ends[min(idx,len(ends))-1]
            idx=min(idx,len(ends)-1)
            direction=p['start_sign']*(-1)**idx
            previous=0. if idx==0 else -direction
            u=blend((elapsed-start)/p['transition']) if elapsed<ends[-1] else 1.
            swing=previous+u*(direction-previous)
            lateral=swing
            envelope=blend(elapsed/.15)
        else:
            phase=2*math.pi*p['hz']*elapsed+p['phase']
            envelope=blend(elapsed/p['rise'])
            swing=envelope*math.tanh(p['sharp']*math.sin(phase))/math.tanh(p['sharp'])
            lateral=envelope*math.sin(phase+p['offset'])
        roll_target=p['amp']*swing+p['bias']*envelope
        if 'durations' in p:
            roll_target=np.clip(roll_target,-40.,40.)
        for scale in (1.,.9,.75,.55,.3,0.):
            pose[:3,:3]=rotation(np.radians([self.side*self.strength*scale*roll_target,scale*p.get('pitch',0.)*envelope,0.]))
            pose[:3,3]=[scale*p.get('x',0.)*envelope/1000,self.side*self.strength*scale*p['y']*lateral/1000,
                         .177+scale*p.get('z',0.)*envelope/1000]
            try:
                self.sim.checked_ik(pose)
                break
            except ValueError:
                if not scale:raise
        self.history.append(self.side*np.array([np.degrees(euler(pose)[0]),1000*pose[1,3]]))
        return pose,'pump',measured


def run(spec):
    bank,side,settings,seed,dt,friction,seconds,clip=spec[:8]
    audit_work=len(spec)>8 and spec[8]
    sim=plant(bank,dt,friction)
    if settings is None:
        ctl=balance_controller(sim,side,dict(seed=seed))
        ctl.hold_from_start=False
    else:
        ctl=RampPump(sim,side,settings,seed)
    work=[0.,0.]
    motors=[i for i in range(sim.model.nu) if sim.model.actuator(i).name.startswith('stewart')]
    def measure(sim,data):
        if data.time<=1.4 or ctl.catch_trigger is None:
            positive=float(np.maximum(data.actuator_force[motors]*data.actuator_velocity[motors],0.).sum())*sim.dt
            if data.time<=1.4:work[0]+=positive
            if ctl.catch_trigger is None:work[1]+=positive
    trial=sim.run_primitive(Primitive(),seconds=seconds,waist_mode='hold_neutral',record=True,
                            tilt_limit_deg=None,observe_landing=True,record_rim_contact=True,controller=ctl,step_audit=measure if audit_work else None)
    rows=trial['trace'];poses=trial['trajectory']
    for r,p in zip(rows,poses):
        r['foot_side_normal_forces_n_right_left']=p['foot_side_normal_forces_n_right_left']
    quiet=rows[:10]
    mean_forces=np.mean([r['foot_side_normal_forces_n_right_left'] for r in quiet],axis=0)
    flat_start=min(mean_forces)>.5 and all(r.get('tilt_from_floor_deg',r['tilt_deg'])<1 and
                                          r['base_speed_m_s']<.02 and not r['edge_support'] for r in quiet)
    load=[];strict=[];soft=[];signed=[];quiet_edge=[];offsets=[]
    bank_rad=math.radians(bank)
    uphill=np.sign(bank)*np.array([0.,math.cos(bank_rad),math.sin(bank_rad)])
    for r in rows:
        f=r['foot_side_normal_forces_n_right_left']
        right,left=f
        directed=r['edge_support'] and (right>left if side>0 else left>right)
        cop=r['contact_world_m']
        offset=float(1000*np.dot(np.asarray(cop)-r['foot_xyz_m'],uphill)) if cop is not None else None
        offsets.append(offset)
        local=r['contact_foot_xy_m']
        directed=directed and local is not None and -side*local[1]>.040
        if bank:
            directed=directed and offset is not None and -np.sign(side*bank)*offset>20.
        a=side*r['roll_deg'];signed.append(a)
        quiet_edge.append(bool(directed and abs(r['roll_rate_deg_s'])<=8 and abs(r['pitch_rate_deg_s'])<=8
                               and r['base_speed_m_s']<.03))
        load.append(bool(directed))
        strict.append(bool(directed and abs(a-32.1)<=2 and abs(r['roll_rate_deg_s'])<=5 and abs(r['pitch_deg'])<=5))
        soft.append(float(directed)*math.exp(-abs(a-32.1)/5-abs(r['roll_rate_deg_s'])/50-abs(r['pitch_deg'])/10))
    contact=trial['first_body_floor_contact']
    driven=[v for v,r in zip(signed,rows) if not contact or r['time_s']<contact['time_s']]
    good=flat_start and trial['completed'] and trial['min_joint_margin_rad']>=0 and not trial['fell']
    edge=good and len(rows)>=26 and all(load[-26:]) and all(abs(v-32.1)<3 for v in signed[-26:]) and all(abs(r['pitch_deg'])<5 for r in rows[-26:])
    strict_pass=good and len(strict)>=251 and all(strict[-251:])
    span=longest=0.
    for flag in quiet_edge:
        span=span+.02 if flag else 0.
        longest=max(longest,span)
    hold=good and len(rows)>=101 and all(quiet_edge[-101:])
    head=np.degrees([r['head_rpy_rad'][0] for r in poses if r['time_s']<=1.4])
    out=dict(bank=bank,side=side,orientation='level' if bank==0 else 'downhill' if side*bank>0 else 'uphill',
             dt=dt,friction=friction,seed=seed,seconds=seconds,elapsed=trial['elapsed_seconds'],flat_start=bool(flat_start),
             start_speed_mm_s=1000*rows[0]['base_speed_m_s'],initial_roll=rows[0]['roll_deg'],initial_force_n=rows[0]['foot_side_normal_forces_n_right_left'],
             strict=bool(strict_pass),edge=bool(edge),hold=bool(hold),longest_quiet_s=max(0.,longest-.02),
             final_contact_upslope_mm=offsets[-1],end_yaw_deg=trial['final_foot_yaw_deg'],fell=trial['fell'],shell=trial['first_body_floor_contact'],
             flat=trial['settled'],joint=trial['min_joint_margin_rad'],termination=trial['termination'],
             peak_requested_lean=max(driven),peak_opposite_lean=max(0.,-min(driven)),final_roll=rows[-1]['roll_deg'],
             loaded_s=.02*sum(load),soft_s=.02*sum(soft),first_edge_s=next((r['time_s'] for r,b in zip(rows,load) if b),None),
             head_span_deg=float(np.ptp(head)),head_peak_deg_s=float(max(abs(np.diff(head)),default=0.)/.02),
             capture_distance=min(math.hypot(v-31.6,r['roll_rate_deg_s']/12.) for v,r in zip(signed,rows)),
             trigger=ctl.catch_trigger,waist=trial['commanded_waist_position_range_rad'],
             sensed_bank=ctl.bank if settings is not None else None)
    if settings is not None and 'stop_s' in settings:
        coast=[(r,p) for r,p in zip(rows,poses) if r['time_s']>=settings['stop_s']+.14
               and (not contact or r['time_s']<contact['time_s'])]
        trim=settings.get('trim',{})
        costs=[math.hypot(side*r['roll_deg']-trim.get('equilibrium',31.55),r['roll_rate_deg_s']/10.)
               +.35*abs(r['pitch_deg'])+.06*abs(r['pitch_rate_deg_s'])
               +.08*abs(side*np.degrees(p['head_rpy_rad'][0])-trim.get('head_roll',2.3)) for r,p in coast]
        # Compare six consecutive samples to distinguish settling from a fast crossing.
        out['coast_cost']=min((max(costs[i:i+6]) for i in range(len(costs)-5)),default=150.)
        out['outward_escape']=bool(contact and driven[-1]>45) or any(side*r['roll_deg']>46 for r,p in coast)
    if audit_work:
        out.update(head_positive_work_first_1_4s_j=work[0],head_positive_work_to_catch_or_end_j=work[1])
    if clip:
        track=[dict(t=r['time_s'],roll=r['roll_deg'],rate=r['roll_rate_deg_s'],lift=r['side_height_difference_mm'],
                    cop=r['contact_foot_xy_m'],edge=l,head=float(np.degrees(p['head_rpy_rad'][0])),qpos=p['qpos']) for r,p,l in zip(rows,poses,load)]
        write(WORK/(clip+'.json'),dict(config=sim.config,result=out,track=track))
    return out


def spec(bank,side,settings=None,seconds=7.,seed=55001,dt=.000125,mu=.7,clip=None):
    return bank,side,settings,seed,dt,mu,seconds,clip


def baseline(pool):
    return list(pool.map(run,[spec(b,s,seconds=6.) for b in (0,3,-3,6,-6,9,-9) for s in (-1,1)]))


def settings(rng):
    return dict(amp=float(rng.uniform(21,40)),y=float(rng.uniform(-24,24)),bias=float(rng.uniform(-7,8)),
                hz=float(rng.uniform(1.65,4.25)),phase=float(rng.uniform(-math.pi,math.pi)),
                offset=float(rng.uniform(-math.pi,math.pi)),rise=float(rng.uniform(.07,.30)),sharp=float(rng.uniform(.45,3.5)),
                lead=float(rng.uniform(.015,.13)),trigger=float(rng.uniform(26.5,33.7)),
                catch_model=str(rng.choice(['plane','full'])))


def grade(r):
    valid=r['flat_start'] and r['termination']=='completed'
    return (100*r['strict']+40*r.get('hold',False)+28*r['edge']+4*r['soft_s']+.1*min(r['peak_requested_lean'],32)
            +.04*min(r['loaded_s'],4)-1.5*r['fell']-1.*(r['joint']<0)-50*(not valid))


def search(pool):
    rng=np.random.default_rng(712431)
    candidates=[settings(rng) for _ in range(58)]
    results=[]
    for bank in (0.,5.,-5.,9.,-9.):
        jobs=[spec(bank,s,p,seconds=2.8,dt=.00025) for s in (-1,1) for p in candidates]
        rows=list(pool.map(run,jobs))
        results.extend(dict(settings=j[2],result=r) for j,r in zip(jobs,rows))
        write(WORK/'search-progress.json',results)
    return results


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('task',choices=['baseline','search'])
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:
        write(WORK/(a.task+'.json'),globals()[a.task](pool))
