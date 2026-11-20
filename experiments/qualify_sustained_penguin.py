"""Expanded, recorded qualification of the sustained penguin controller."""
import argparse,copy,json
from pathlib import Path
import numpy as np
from train_sustained_penguin import batch,development,read
from train_faster_penguin import make_job

HARD=[('fresh-3',-1),('fresh-5',1),('fresh-7',-1),('fresh-7',1),('unseen-3',-1),('unseen-7',-1)]

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['development','fresh','fine','long'])
    parser.add_argument('--policy',required=True);parser.add_argument('--name',required=True)
    parser.add_argument('--workers',type=int,default=4);parser.add_argument('--seed',type=int,default=2026112007)
    args=parser.parse_args();policy=read(args.policy);sources=development();nominal=sources[0];jobs=[]
    if args.mode=='development':
        for source in sources:
            j=make_job(source,policy,seconds=26);j['finish_settle_s']=1.;j['variant']='single-pair';jobs.append(j)
        for source in sources:
            if (source['id'],source['side']) in HARD:
                j=make_job(source,policy,seconds=100,cycles=4);j['finish_settle_s']=1.;j['variant']='four-pairs';jobs.append(j)
    elif args.mode=='fresh':
        rng=np.random.default_rng(args.seed)
        for i in range(8):
            source=copy.deepcopy(nominal);source['id']=f'heldout-{i}'
            source['config']['friction_slide_torsion_roll'][0]=float(rng.uniform(.55,.9))
            source['config']['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),
                       delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
            for side in [-1,1]:
                j=make_job(source,policy,seconds=100,cycles=4);j['finish_settle_s']=1.;j['side']=side;j['variant']='heldout-four-pairs'
                j['heading_seed']=int(rng.integers(0,100000))
                sensor=dict(sensor_delay_frames=int(rng.integers(1,3)),sensor_noise_deg=float(rng.uniform(.05,.15)),sensor_seed=int(rng.integers(0,100000)))
                for family in ['primary','fallback','delayed_sensor_fallback']:j['policy'][family]['feedback'].update(sensor)
                jobs.append(j)
    elif args.mode=='fine':
        for source in sources:
            if (source['id'],source['side']) in HARD:
                j=make_job(source,policy,seconds=26,dt=.000125);j['variant']='finer-single-pair';jobs.append(j)
    else:
        for side,dt in [(-1,.00025),(1,.00025),(-1,.000125),(1,.000125)]:
            j=make_job(nominal,policy,seconds=200,cycles=20,dt=dt);j['side']=side;j['variant']=f'twenty-pairs-{dt}'
            if dt==.00025:j['record']=f'experiments/generated/sustained-penguin/final-twenty-{side}.json'
            jobs.append(j)
    batch(jobs,args.name,args.workers,dict(frozen_policy=policy,qualification_mode=args.mode,
        seed=args.seed if args.mode=='fresh' else None,holdout_not_used_for_selection=args.mode=='fresh'))
