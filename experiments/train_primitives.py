"""Bounded local CPU experiments for tipping, recovery and edge pivots."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.primitives import Primitive,PrimitiveSimulator

CACHE={}

def read(p):return json.loads(Path(p).read_text())

def evaluate(job):
    name,recipe,waist,dt,seconds=job
    config=read('configs/motors-speed-trained.json');config['timestep_s']=dt
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    r=CACHE[key].run_primitive(recipe,seconds=seconds,waist_mode=waist)
    r.pop('trace')
    return dict(id=name,recipe=recipe,waist_mode=waist,timestep_s=dt,result=r)

def summary(c):
    r=c['result']
    return dict(id=c['id'],termination=r['termination'],peak_tilt_deg=r['peak_tilt_deg'],
                peak_roll_deg=r['peak_roll_deg'],lift_mm=r['peak_side_height_difference_mm'],
                recovered=r['recovered'],foot_yaw_deg=r['final_foot_yaw_deg'],
                torso_yaw_deg=r['final_torso_yaw_deg'],edge_yaw_deg=r['edge_supported_foot_yaw_deg'])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['map','catch','pivot'],required=True)
    p.add_argument('--input');p.add_argument('--output',required=True)
    p.add_argument('--waist',choices=['off','hold_neutral'],default='off')
    p.add_argument('--timestep',type=float,choices=[.001,.0005,.00025],default=.001)
    p.add_argument('--seconds',type=float,default=6.)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve previous experiments')
    jobs=[];rng=np.random.default_rng(2026111950);seeds=[]
    if args.mode=='map':
        neutral=asdict(Primitive(preload_roll_deg=0,roll_deg=0))
        jobs.append(('neutral',neutral,args.waist,args.timestep,args.seconds))
        for i,(roll,y,phase,hz) in enumerate(itertools.product([30,38],[12,24],[-90,0,90,180],[2,3,4.5])):
            g=asdict(Primitive(preload_roll_deg=-roll,roll_deg=roll,preload_y_mm=-y,y_mm=y,
                               pump_cycles=8,pump_hz=hz,pump_y_phase_deg=phase,pump_sharpness=1.5))
            jobs.append((f'pump-{i}',g,args.waist,args.timestep,args.seconds))
        for i in range(48):
            roll=float(rng.uniform(20,39));y=float(rng.uniform(-24,24));side=(-1 if i%2 else 1)
            g=asdict(Primitive(preload_roll_deg=-side*roll,roll_deg=side*roll,preload_y_mm=-y,y_mm=y,
                               pitch_deg=float(rng.uniform(-10,12)),x_mm=float(rng.uniform(-15,15)),z_mm=float(rng.uniform(-8,8)),
                               pump_cycles=8,pump_hz=float(rng.uniform(1.5,4.9)),pump_y_phase_deg=float(rng.uniform(-180,180)),
                               pump_sharpness=float(rng.uniform(0,3.5))))
            jobs.append((f'random-{i}',g,args.waist,args.timestep,args.seconds))
    else:
        source=read(args.input)
        if not source['completed']:p.error('Wait for the source study to finish')
        valid=[c for c in source['candidates'] if c['result']['model_valid']]
        if args.mode=='catch':
            valid.sort(key=lambda c:(c['result']['fell'],c['result']['peak_roll_deg']),reverse=True)
            seeds=valid[:4]
            for i,c in enumerate(seeds):
                peak=c['result']['peak_roll_deg']
                thresholds=[x for x in [6,10,14,18,22,26,30,34,38] if x<=.95*peak]
                for threshold,action,duration in itertools.product(thresholds,['neutral','preload','continue'],[.06,.16,.32]):
                    g=copy.deepcopy(c['recipe']);g.update(catch_at_deg=threshold,catch_s=duration)
                    if action=='preload':g.update(catch_roll_deg=g['preload_roll_deg'],catch_y_mm=g['preload_y_mm'])
                    elif action=='continue':g.update(catch_roll_deg=g['roll_deg'],catch_y_mm=g['y_mm'])
                    jobs.append((f'{i}-{threshold}-{action}-{duration}',g,args.waist,args.timestep,args.seconds))
        else:
            valid=[c for c in valid if c['result']['recovered']]
            valid.sort(key=lambda c:c['result']['peak_roll_deg']-.5*1000*np.linalg.norm(c['result']['planar_displacement_m']),reverse=True)
            seeds=valid[:3]
            for i,c in enumerate(seeds):
                for yaw,trigger,duration in itertools.product([-50,-30,30,50],[4,8,12,18],[.06,.16]):
                    g=copy.deepcopy(c['recipe']);g.update(yaw_deg=yaw,yaw_at_roll_deg=trigger,yaw_s=duration,yaw_hold_s=.18)
                    jobs.append((f'{i}-yaw{yaw}-at{trigger}-s{duration}',g,args.waist,args.timestep,args.seconds))
            if not jobs:p.error('No recovered lean to build a pivot on; preserve earlier evidence')
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                objective={'map':'Measure sideways tipping and reachable lift; deliberate diagnostic falls permitted',
                           'catch':'Recover a large lean; matched no-catch baseline distinguishes recovery from an arrested fall',
                           'pivot':'Rotate the contacting foot, not just the passive waist, and return upright'}[args.mode],
                selected_seeds=[dict(id=c['id'],recipe=c['recipe'],result=c['result']) for c in seeds],
                candidates=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('src/reachy_locomotion/motors.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();print(json.dumps(summary(c)),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
