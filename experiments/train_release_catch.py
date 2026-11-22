"""Catch CAD-foot releases using delayed attitude, gyro, and measured head state."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from reachy_locomotion.edge_balance import balance_metrics
from reachy_locomotion.release_plane import ReleasePlane, ReleaseSelector
from reachy_locomotion.primitives import Primitive, blend
from tilted_release import plant, balance_controller, HeadMotion, read, write

WORK = Path('experiments/generated/release-catch')
POLICY = Path('configs/cad-release-catch.json')
OLD = {'left':dict(gain=.9,rate_limit=120,pitch_bias=2.,pitch_gain=.7),'right':{}}
# Generated independently of the training conditions. Both controllers see identical releases.
RESERVED = ((30.15,1.4),(30.65,-1.8),(31.2,0.),(31.7,2.2),(31.95,-1.2),
            (32.15,1.6),(32.55,-2.1),(33.1,0.),(33.65,.9),(33.9,-1.5))


class CatchTrim:
    """Move the regulator's roll reference smoothly from measured startup to its trim."""
    def __init__(self, child, params):
        self.child = child
        self.params = params
        self.release = None
        self.initial_gain = child.control_gain
        self.initial_pitch = child.xeq[1]
        self.original_bias = child.ueq.copy()

    def __getattr__(self,name):
        return getattr(self.child,name)

    def target(self,t,roll):
        c,p = self.child,self.params
        if self.release is None:
            # The first attitude sample is acquired during the supported setup.
            self.release = c.side*(roll+c.rng.normal(0,c.f.sensor_noise_deg))-32.1
        u = blend(t/p.get('ramp',.5))
        c.xeq[0] = p.get('trim',0.) + (1-u)*p.get('follow',0.)*self.release
        c.xeq[1] = self.initial_pitch
        c.capture_gain = c.control_gain = self.initial_gain*(1+(1-u)*(p.get('onset_gain',1.)-1))
        c.ueq[:] = self.original_bias
        c.ueq[3] += p.get('y_bias',0.)
        return c.target(t,roll)


def make_controller(sim,side,seed,policy):
    def controller(p):
        if p is None:
            return HeadMotion(dict(mode='hold', side=side))
        if p.get('plane'):
            return ReleasePlane(side,seed,sim.checked_ik,p)
        child=balance_controller(sim,side,dict(p,seed=seed))
        return CatchTrim(child,p) if p.get('adaptive') else child
    if policy and 'specialists' in policy:
        return ReleaseSelector(side,seed,policy,controller)
    return controller(policy)


def simulate(job):
    side,angle,rate,seed,dt,seconds,policy,clip = job[:8]
    environment=job[8] if len(job)>8 else {}
    sim,neutral=plant(True,dt,environment.get('reduced_bus', False),environment.get('sliding_friction'))
    sim.initial_targets=neutral.copy()
    ctl=make_controller(sim,side,seed,policy)
    r=sim.run_primitive(Primitive(),seconds=seconds,controller=ctl,waist_mode='hold_neutral',
        tilt_limit_deg=None,observe_landing=True,record=bool(clip),record_rim_contact=True,
        diagnostic_initial_roll_deg=side*angle,diagnostic_initial_angular_rate_deg_s=side*rate)
    rows=r['trace']
    b=balance_metrics(r,side,32.1)
    safe=r['completed'] and r['min_joint_margin_rad']>=0 and not r['fell']
    end=rows[-26:]
    on_edge=bool(safe and all(x['edge_support'] and abs(side*x['roll_deg']-32.1)<=3 and abs(x['pitch_deg'])<=5 for x in end))
    outcome=('shell landed' if r['first_body_floor_contact'] else 'flat' if r['settled'] else
             'edge' if on_edge else 'unresolved')
    result=dict(side=side,angle=angle,outward_rate=rate,seed=seed,dt=dt,seconds=seconds,
                termination=r['termination'],elapsed=r['elapsed_seconds'],outcome=outcome,
                strict=bool(safe and b['final_five_seconds_pass']),edge=on_edge,balance=b,
                safe_flat=bool(safe and r['settled']),shell=r['first_body_floor_contact'],
                joint=r['min_joint_margin_rad'],waist=r['commanded_waist_position_range_rad'],
                tail_error=float(np.mean([abs(side*x['roll_deg']-32.1)+abs(x['pitch_deg'])*.3 for x in end])))
    if environment:
        result['environment']=environment
    if isinstance(ctl,ReleaseSelector):
        result['release_bin']=ctl.index
    if clip:
        keys=('time_s','roll_deg','roll_rate_deg_s','pitch_deg','contact_world_m','contact_foot_xy_m','com_world_m','normal_n','edge_support','foot_xyz_m','edge_action','edge_observation','plane_action','plane_state')
        trace=[{k:x[k] for k in keys if k in x} for x in rows]
        write(WORK/(clip+'.json'),dict(config=r['config'],trace=trace,trajectory=r['trajectory'],result=result))
    return result


def batch(pool,jobs):
    return list(pool.map(simulate,jobs))


def job(side,angle,rate,policy,seconds,seed=4915,dt=.000125,clip=None):
    return side,angle,rate,seed,dt,seconds,policy,clip


def paired(pool):
    labels=('original-left','original-right','still-left','still-right','same-left')
    recipes=(OLD['left'],OLD['right'],None,None,dict(regulator_side=1))
    return batch(pool,[job(s,32.2,0.,p,8.,clip=k) for s,p,k in zip((-1,1,-1,1,-1),recipes,labels)])


def score(results):
    return (sum(r['strict'] for r in results),sum(r['edge'] for r in results),
            sum(r['safe_flat'] for r in results),sum(min(r['balance']['longest_strict_hold_s'],8) for r in results),
            -sum(min(r['tail_error'],50.) for r in results))


def screen(pool):
    rng=np.random.default_rng(50123)
    candidates=[]
    for i in range(72):
        candidates.append(dict(adaptive=True,gain=float(rng.uniform(.45,1.25)),rate_limit=int(rng.choice([60,100,150,220,360])),
            trim=float(rng.uniform(-1.2,1.2)),pitch_bias=float(rng.uniform(-2.8,2.8)),
            pitch_gain=float(rng.uniform(.4,1.5)),follow=float(rng.uniform(.2,1.2)),ramp=float(rng.uniform(.15,.9)),
            onset_gain=float(rng.uniform(.35,1.25)),y_bias=float(rng.uniform(-2,2))))
    rows=batch(pool,[job(1,32.2,0.,p,2.) for p in candidates])
    return [dict(policy=p,results=[r]) for p,r in zip(candidates,rows)]


def ranked(rows):
    return sorted(rows,key=lambda x:score(x['results']),reverse=True)


def compare(pool, candidates, settings, seconds):
    rows=[]
    jobs=[job(s,a,v,p,seconds,seed) for s,p in candidates for a,v,seed in settings]
    results=batch(pool,jobs)
    n=len(settings)
    for i,(side,p) in enumerate(candidates):
        rows.append(dict(side=side,policy=p,results=results[i*n:(i+1)*n]))
    return rows


def plane_screen(pool):
    from fit_release_plane import design
    fitted=read(WORK/'identification.json')['models']
    rng=np.random.default_rng(5992)
    candidates=[]
    for i in range(48):
        weight=float(10**rng.uniform(-.1,2.4))
        cost=float(10**rng.uniform(-.65,.6))
        trim=float(rng.uniform(-.9,.9))
        gain=float(rng.uniform(.65,1.25))
        rate=float(rng.choice([100,160,250,400,650,900]))
        for s in (1,-1):
            model=design(fitted[str(s)],weight,trim,cost)
            candidates.append((s,dict(plane=True,weight=weight,cost=cost,trim=trim,gain=gain,rate=rate,model=model)))
    return compare(pool,candidates,[(32.2,0.,4915)],2.4)


def varied(pool):
    screened=read(WORK/'plane_screen.json')
    original=ranked(read(WORK/'screen.json'))
    candidates=[(1,c['policy']) for c in original[:6]]
    for s in (1,-1):
        candidates.append((s,OLD['left' if s==-1 else 'right']))
        candidates.extend((s,c['policy']) for c in ranked([v for v in screened if v['side']==s])[:7])
    return compare(pool,candidates,[(30.,1.,5051),(31.,-1.,5052),(32.2,0.,4915),(33.,1.,5053),(34.,-1.,5054)],2.8)


def finalists(pool):
    rows=read(WORK/'varied.json')
    candidates=[(s,c['policy']) for s in (1,-1) for c in ranked([v for v in rows if v['side']==s])[:4]]
    return compare(pool,candidates,[(30.,2.,6111),(31.25,-2.,6112),(32.2,0.,4915),(32.75,1.,6113),(34.,-1.,6114)],8.)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('phase',choices=['paired','screen','plane_screen','varied','finalists'])
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:
        write(WORK/(a.phase+'.json'),globals()[a.phase](pool))
