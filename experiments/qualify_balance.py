"""Frozen-policy checks on both sides, finer physics and unseen model draws."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.balance import BalanceController,mirrored
from reachy_locomotion.primitives import PrimitiveSimulator
from train_balance import read,models,recovery_score
from train_balance_pivot import groups,signed_pivot
from qualify_primitives import pivot_metrics

CACHE={}
def evaluate(job):
    label,g,f,side,config=job
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    recipe=mirrored(g,side);controller=BalanceController(recipe,f,checked_ik=CACHE[key].checked_ik)
    r=CACHE[key].run_primitive(recipe,seconds=5,waist_mode='hold_neutral',controller=controller,tilt_limit_deg=None)
    r.pop('trace');r.pop('terminal_state')
    r['selected_yaw_target_deg']=controller.yaw_amplitude
    return dict(id=label,recipe=g,feedback=f,side=side,result=r,pivot_metrics=pivot_metrics(r))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--kind',choices=['recovery','pivot'],required=True)
    p.add_argument('--seed',type=int,default=2026111961)
    p.add_argument('--candidate-id')
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for complete source')
    ranked=groups(source)
    if args.kind=='recovery':ranked.sort(key=lambda cs:(sum(c['result']['recovered'] for c in cs),min(recovery_score(c['result']) for c in cs)),reverse=True)
    else:ranked.sort(key=lambda cs:(sum(signed_pivot(c) for c in cs),min(c['pivot_metrics']['score'] for c in cs)),reverse=True)
    selected=next(cs[0] for cs in ranked if not args.candidate_id or cs[0]['id']==args.candidate_id)
    g=selected['recipe'];f=selected['feedback'];jobs=[]
    for i,config in enumerate(models()):
        for side in [-1,1]:
            c=copy.deepcopy(config);c['timestep_s']=.00025;jobs.append((f'development-{i}',g,f,side,c))
    rng=np.random.default_rng(args.seed)
    for i in range(8):
        c=copy.deepcopy(models()[0]);c['timestep_s']=.00025
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.55,.9))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
        for side in [-1,1]:
            sensor=dict(f,sensor_delay_frames=int(rng.integers(1,3)),sensor_noise_deg=float(rng.uniform(.05,.15)),sensor_seed=int(rng.integers(0,100000)))
            jobs.append((f'fresh-{i}',g,sensor,side,c))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),frozen_candidate=selected,
                frozen_jobs=[dict(id=j[0],recipe=j[1],feedback=j[2],side=j[3],config=j[4]) for j in jobs],checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],recovered=r['recovered'],pivot_pass=signed_pivot(c),roll=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
