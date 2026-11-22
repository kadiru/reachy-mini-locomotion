"""Release a hanging ball from several angles against a naturally rolling Reachy."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy

import mujoco
import numpy as np

from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.side_pendulum import SwingContacts
from train_deep_lift import WORK, POLICY, LiftCircle, simulator, read, write

RELEASE = 5.
LENGTH = .62
RADIUS = .025
CLEARANCE = .2032
HEIGHT = .115
ANGLES = (0,10,15,20,25,30,45)


def fixture(side):
    sim=simulator('edge_nominal')
    d=mujoco.MjData(sim.model)
    d.qpos[:]=sim.initial_qpos
    mujoco.mj_forward(sim.model,d)
    # Ray at the bob's resting height to the upright robot's physical shell.
    geom=np.array([-1],dtype=np.int32)
    mask=np.array([0,0,0,1,0,0],dtype=np.uint8)
    dist=mujoco.mj_ray(sim.model,d,np.array([0.,float(side),HEIGHT]),np.array([0.,-float(side),0.]),
                        mask,True,-1,geom)
    if dist<0 or sim.model.body_rootid[sim.model.geom_bodyid[geom[0]]]!=sim.base:
        raise ValueError('Rest-clearance ray missed the robot shell')
    edge=1-dist
    return dict(length_m=LENGTH,radius_m=RADIUS,mass_kg=.025,side=side,release_s=RELEASE,
                anchor_xyz_m=[0.,side*(edge+CLEARANCE+RADIUS),HEIGHT+LENGTH],
                shell_ray_m=edge,rest_surface_gap_m=CLEARANCE)


def swing(job):
    angle,side,policy,record,*duration=job
    seconds=duration[0] if duration else 12.
    setting=dict(fixture(side),release_deg=angle)
    config=deepcopy(simulator('edge_nominal').config)
    config['side_pendulum']=setting
    sim=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    ctl=LiftCircle(sim,'edge_nominal',policy['lift'],policy['orbit'])
    audit=SwingContacts(sim.model,RELEASE)
    run=sim.run_primitive(Primitive(),seconds=seconds,controller=ctl,waist_mode='hold_neutral',tilt_limit_deg=None,
                          record=record,record_rim_contact=True,step_audit=audit,observe_landing=True)
    trace=run['trace']
    before=trace[round(RELEASE/.02)-1]
    tail=trace[-100:]
    tail_angles=np.array([row['tilt_deg'] for row in tail])
    captured=ctl.start is not None and ctl.start<RELEASE
    joint=run['first_joint_travel_warning']
    floor=run['first_body_floor_contact']
    first=audit.hits[0] if audit.hits else None
    last_hit=audit.hits[-1]['last_s'] if first else None
    if first and run['completed'] and not floor and seconds<20 and seconds-last_hit<2.1:
        end=min(20.,float(np.ceil((last_hit+3.1)*50)/50))
        return swing((angle,side,policy,record,end))
    supported=np.array([r['normal_n']>.5 and r['contact_foot_xy_m'] is not None and
                        np.linalg.norm(r['contact_foot_xy_m'])>=.85*config['foot_radius_m'] for r in tail])
    recovered=bool(captured and first and run['completed'] and not floor and run['min_joint_margin_rad']>=0 and
                   tail[0]['time_s']>last_hit and 29<=tail_angles[-1]<=33 and supported[-1] and
                   np.mean((tail_angles>=29)&(tail_angles<=33)&supported)>=.95)
    result=dict(settings=setting,capture_s=ctl.start,captured_before_release=captured,
                lean_before_release_deg=before['tilt_deg'],hits=audit.hits,bob_floor_s=audit.floor_s,
                cable_length_range_m=audit.lengths,termination=run['termination'],body_floor=floor,
                joint_event=joint,joint_margin_rad=run['min_joint_margin_rad'],waist_rad=run['commanded_waist_position_range_rad'],
                final_lean_deg=float(tail_angles[-1]),tail_lean_range_deg=[float(tail_angles.min()),float(tail_angles.max())],
                recovered=recovered,elapsed_s=run['elapsed_seconds'])
    if record:
        write(WORK/f'pendulum-{side}-{angle}.json',
              dict(trace=trace,trajectory=[x['qpos'] for x in run['trajectory']],metrics=result,config=config))
    print('swing',side,angle,captured,len(audit.hits),recovered,result['termination'],flush=True)
    return result


def bracket_hits(rows,policy,workers):
    extra=[]
    if any(r['recovered'] for r in rows):
        return extra
    for _ in range(2):
        jobs=[]
        for side in (1,-1):
            seen=[r for r in rows+extra if r['settings']['side']==side]
            hit_angles=[r['settings']['release_deg'] for r in seen if r['hits']]
            if not hit_angles:
                continue
            high=min(hit_angles)
            misses=[r['settings']['release_deg'] for r in seen if not r['hits'] and r['settings']['release_deg']<high]
            if misses:
                low=max(misses)
                jobs.extend((low+(high-low)*fraction,side,policy,True) for fraction in (.25,.5,.75))
        if not jobs:
            break
        with ProcessPoolExecutor(workers) as pool:
            extra.extend(pool.map(swing,jobs))
        if any(r['recovered'] for r in extra):
            break
    return extra


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=6)
    parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--bracket',action='store_true')
    args=parser.parse_args()
    p=read(POLICY) if POLICY.exists() else dict(lift={},orbit={})
    if args.smoke:
        write(WORK/'pendulum-smoke.json',swing((30,1,p,True)))
    elif args.bracket:
        rows=read(WORK/'pendulum-checks.json')
        late=[i for i,r in enumerate(rows) if r['hits'] and not r['body_floor'] and
              (r['recovered'] or r['elapsed_s']-r['hits'][-1]['last_s']<2.1)]
        jobs=[(rows[i]['settings']['release_deg'],rows[i]['settings']['side'],p,True,
               min(20.,max(round(rows[i]['elapsed_s'],2),float(np.ceil((rows[i]['hits'][-1]['last_s']+3.1)*50)/50)))) for i in late]
        if jobs:
            with ProcessPoolExecutor(args.workers) as pool:
                for i,result in zip(late,pool.map(swing,jobs)):
                    rows[i]=result
            write(WORK/'pendulum-checks.json',rows)
        write(WORK/'pendulum-grazing.json',bracket_hits(rows,p,args.workers))
    else:
        jobs=[(a,s,p,True) for s in (1,-1) for a in ANGLES]
        with ProcessPoolExecutor(args.workers) as pool:
            results=list(pool.map(swing,jobs))
        write(WORK/'pendulum-checks.json',results)
