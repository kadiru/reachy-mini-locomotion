"""Rotate the reference of the natural side-edge balance regulator around the foot."""
import argparse
import math
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from reachy_locomotion.edge_linear import LinearEdgeController
from reachy_locomotion.geometry import rotation
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.rim_roll import measures, ground_course
from train_gyro_peg import MODELS,WORK,simulator,read,write

# The natural capture was identified at this integration rate and sensor setting.
MODELS.update(edge_nominal=(.7,5.,1.,1,'torso',.000125,1,0.,0),
              edge_fine=(.7,5.,1.,1,'torso',.0001,1,0.,0),
              edge_floor=(.67,5.,1.,1,'torso',.000125,1,0.,0),
              edge_battery=(.7,4.95,.99,1,'torso',.000125,1,0.,0),
              edge_noise=(.7,5.,1.,1,'torso',.000125,1,.03,41))


class TravelingEdge:
    def __init__(self,sim,side,period,direction,gain,condition,sector_mix=0.):
        self.sim=sim
        self.side,self.period,self.direction,self.gain=side,period,direction,gain
        p=read('configs/edge-balance-experimental.json')['sides']['left' if side<0 else 'right']
        feedback=dict(p['feedback'],sensor_noise_deg=MODELS[condition][-2],sensor_seed=MODELS[condition][-1])
        self.child=LinearEdgeController(Primitive(**p['recipe']),feedback,side,self.check_virtual,p['linear'])
        self.gain0=p.get('linear_gain_scale',1.)
        self.child.control_gain=self.gain0
        self.start=None
        self.stable=0
        self.Q=np.eye(3)
        self.angle=0.
        self.pose=None
        self.pitch=0.

        self.sector_mix=sector_mix
        if sector_mix:
            # Express each calibrated side or fore/aft model in one traveling
            # frame. Opposite tangential axes reverse pitch and fore/aft shift.
            sides=read('configs/edge-balance-experimental.json')['sides']
            fore=read('configs/foreaft-balance-experimental.json')['edges']
            banks=[(sides['right'],1),(fore['front'],1),(sides['left'],-1),(fore['back'],-1)]
            if side<0:banks=banks[2:]+banks[:2]
            self.banks=[]
            for bank,source_side in banks:
                h=side*source_side
                u=np.array([1.,h,h,1.])
                pos=np.array([1.,h,1.,h,h,1.])
                state=np.r_[pos,pos,u,u]
                model=bank['linear']
                self.banks.append(dict(A=state[:,None]*np.array(model['A'])*state,
                                       B=state[:,None]*np.array(model['B'])*u,
                                       c=state*np.array(model['c']),
                                       K=u[:,None]*np.array(model['gain'])*state,
                                       xeq=state*np.array(model['x_equilibrium']),
                                       ueq=u*np.array(model['u_equilibrium'])))

    def mix_models(self):
        quadrant=(self.angle/(math.pi/2))%4
        first=int(quadrant)
        blend=quadrant-first
        blend=blend*blend*(3-2*blend)
        for k,v in self.banks[0].items():
            mixed=(1-blend)*self.banks[first][k]+blend*self.banks[(first+1)%4][k]
            setattr(self.child,k,(1-self.sector_mix)*v+self.sector_mix*mixed)

    @property
    def yaw_trigger(self):return self.child.yaw_trigger

    @property
    def catch_trigger(self):return self.child.catch_trigger

    def physical(self,p):
        if not self.angle:return p
        out=p.copy()
        out[:3,:3]=self.Q@p[:3,:3]@self.Q.T
        out[:3,3]=self.Q@p[:3,3]
        return out

    def check_virtual(self,p):
        target=self.physical(p)
        if np.max(np.abs(target[:2,3]))>.0250001:
            raise ValueError('Physical translation exceeds head workspace')
        return self.sim.checked_ik(target)

    def observe_head_state(self,t,pose):self.pose=pose

    def observe_attitude(self,t,pitch):self.pitch=pitch

    def reference_angle(self,elapsed):
        return self.direction*(2*math.pi/self.period)*(elapsed-1.+math.exp(-elapsed))

    def target(self,t,roll):
        if self.start is None:
            stable=(self.child.catch_trigger is not None and abs(self.child.sensed[0]-self.side*self.child.f.target_roll_deg)<2.
                    and abs(self.child.rate[0])<5 and abs(self.pitch)<5)
            self.stable=self.stable+1 if stable else 0
            if self.stable>=25:self.start=t
        if self.start is not None:
            dt=t-self.start
            self.angle=self.reference_angle(dt)
            self.Q=rotation((0,0,self.angle))
            if self.sector_mix:self.mix_models()
            self.child.control_gain=self.gain0*(1.+(self.gain-1.)*min(1.,dt))
        pitch=self.pitch
        pose=self.pose
        if self.angle:
            p=pose.copy()
            p[:3,:3]=self.Q.T@pose[:3,:3]@self.Q
            p[:3,3]=self.Q.T@pose[:3,3]
            pose=p
            gravity=self.Q.T@rotation(np.radians([roll,pitch,0]))[2,:]
            roll,pitch=np.degrees([math.atan2(gravity[1],gravity[2]),math.asin(np.clip(-gravity[0],-1,1))])
        self.child.observe_head_state(t,pose)
        self.child.observe_attitude(t,pitch)
        target,stage,sensed=self.child.target(t,roll)
        return self.physical(target),stage,sensed


def trial(job,sector_mix=0.,step_audit=None):
    side,period,direction,gain,condition,seconds,*record=job
    sim=simulator(condition)
    ctl=TravelingEdge(sim,side,period,direction,gain,condition,sector_mix)
    run=sim.run_primitive(Primitive(),seconds=seconds,controller=ctl,waist_mode='hold_neutral',
                          tilt_limit_deg=None,record=bool(record),record_rim_contact=True,step_audit=step_audit)
    m=measures(run['trace'],run['config']['foot_radius_m'],direction,start_s=ctl.start or 1.5)
    result=dict(side=side,period=period,direction=direction,gain=gain,condition=condition,seconds=seconds,
                captured_s=ctl.start,elapsed_s=run['elapsed_seconds'],termination=run['termination'],fell=run['fell'],
                safe=run['completed'] and run['min_joint_margin_rad']>=0,waist_command_rad=run['commanded_waist_position_range_rad'],
                base_yaw_deg=run['final_foot_yaw_deg'],joint_margin_rad=run['min_joint_margin_rad'],metrics=m)
    if sector_mix:result['sector_mix']=sector_mix
    result['passed']=bool(result['safe'] and ctl.start is not None and m['longest_rim_turns']>=2.
                          and m['median_lean_deg']>=25. and m['loaded_fraction']>=.95
                          and m['coherent_fraction']>=.9 and m['slip_per_rim_arc']<=.25)
    if seconds>=24. and len(run['trace'])>210:
        result['floor_course']=ground_course(run['trace'],2000*run['config']['foot_radius_m'])
    if record:write(record[0],dict(trace=run['trace'],config=run['config'],metrics=result,terminal_state=run['terminal_state'],trajectory=[x['qpos'] for x in run['trajectory']]))
    return result


def fit(n):
    jobs=[(side,p,d,g,'edge_nominal',9.) for side in (-1,1) for p in (2.5,5.,10.,20.,40.)
          for d in (-1,1) for g in (1.,)]
    with ProcessPoolExecutor(n) as pool:rows=list(pool.map(trial,jobs))
    write(WORK/'traveling-edge.json',rows)


def endurance(n):
    # Only the first nine nominal seconds were used to nominate these trials.
    rows=read(WORK/'traveling-edge.json')
    winners=sorted([r for r in rows if r['safe'] and r['captured_s'] is not None],
                   key=lambda r:r['metrics']['longest_rim_turns'],reverse=True)[:6]
    jobs=[(r['side'],r['period'],r['direction'],r['gain'],'edge_nominal',4.5+2.3*r['period']) for r in winners]
    with ProcessPoolExecutor(n) as pool:checks=list(pool.map(trial,jobs))
    write(WORK/'traveling-endurance.json',checks)
    best=max(checks,key=lambda r:(r['safe'] and r['metrics']['laps']>=1,r['metrics']['longest_rim_turns']))
    p=tuple(best[k] for k in ('side','period','direction','gain'))
    write(WORK/'deep-frozen.json',dict(side=p[0],period=p[1],direction=p[2],gain=p[3],seconds=best['seconds']))
    print(best,flush=True)


def qualify(n):
    frozen=read(WORK/'deep-frozen.json')
    p=tuple(frozen[k] for k in ('side','period','direction','gain'))
    seconds=frozen['seconds']
    nom=trial((*p,'edge_nominal',seconds,WORK/'deep-best.json'))
    jobs=[(*p,c,seconds) for c in ('edge_floor','edge_battery','edge_noise','edge_fine')]
    with ProcessPoolExecutor(n) as pool:checks=list(pool.map(trial,jobs))
    write(WORK/'deep-checks.json',dict(nominal=nom,held_out=checks))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workers',type=int,default=6)
    p.add_argument('phase',nargs='?',default='fit',choices=('fit','endurance','qualify'))
    a=p.parse_args()
    {'fit':fit,'endurance':endurance,'qualify':qualify}[a.phase](a.workers)
