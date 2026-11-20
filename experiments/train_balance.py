"""Train bilateral roll/rate correction on fixed, disclosed model conditions."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from reachy_locomotion.balance import BalanceController,BalanceFeedback,mirrored
from reachy_locomotion.primitives import PrimitiveSimulator
from qualify_primitives import pivot_metrics

CACHE={}
def read(p):return json.loads(Path(p).read_text())

def models():
    nominal=read('configs/motors-speed-trained.json')
    old=read('experiments/reports/motor-training-primitives-final-2026-11-19.json')
    return [nominal]+[c['result']['config'] for c in old['sensitivity'] if c['id'].startswith('catch')]

def evaluate(job):
    label,recipe,feedback,side,model_id,dt,seconds=job
    config=copy.deepcopy(models()[model_id]);config['timestep_s']=dt
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    g=mirrored(recipe,side);c=BalanceController(g,feedback,checked_ik=CACHE[key].checked_ik)
    r=CACHE[key].run_primitive(g,seconds=seconds,waist_mode='hold_neutral',controller=c,tilt_limit_deg=35)
    trace=r.pop('trace');r.pop('terminal_state')
    r['max_feedback_correction_deg']=max(abs(x['feedback_roll_correction_deg']) for x in trace)
    r['signed_peak_roll_deg']=[min(x['roll_deg'] for x in trace),max(x['roll_deg'] for x in trace)]
    r['balance_feedback']=feedback
    r['workspace_projected_frames']=c.projected_frames
    r['selected_yaw_target_deg']=c.yaw_amplitude
    return dict(id=label,recipe=recipe,feedback=feedback,side=side,model_id=model_id,timestep_s=dt,result=r,pivot_metrics=pivot_metrics(r))

def recovery_score(r):
    if not r['completed']:return -200-max(0,r['peak_tilt_deg']-35)
    if not r['settled']:return -30
    return (100 if r['recovered'] else 0)+min(r['peak_roll_deg'],18)-.2*1000*np.linalg.norm(r['planar_displacement_m'])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['probe','stress'],required=True)
    p.add_argument('--input');p.add_argument('--output',required=True)
    p.add_argument('--timestep',type=float,default=.001);p.add_argument('--seconds',type=float,default=5)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve previous evidence')
    jobs=[]
    if args.mode=='probe':
        for family in ['catch','pivot']:
            g=read(f'configs/primitive-{family}-experimental.json');g['yaw_deg']=0
            for i,(kp,kd,yr,pred) in enumerate(itertools.product([-1.5,0,1.5],[-.15,0,.15],[-.6,.6],[.06])):
                f=asdict(BalanceFeedback(roll_kp=kp,roll_kd_s=kd,y_per_roll_mm_deg=yr,prediction_s=pred,trigger_deg=8))
                for side in [-1,1]:jobs.append((f'{family}-{i}',g,f,side,0,args.timestep,args.seconds))
    else:
        source=read(args.input);groups={}
        for c in source['candidates']:groups.setdefault(c['id'],[]).append(c)
        ranked=sorted(groups.values(),key=lambda cs:min(recovery_score(c['result']) for c in cs),reverse=True)
        for cs in ranked[:8]:
            c=cs[0]
            for side,model in itertools.product([-1,1],range(5)):
                jobs.append((c['id'],c['recipe'],c['feedback'],side,model,args.timestep,args.seconds))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),candidates=[],
                source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path('src/reachy_locomotion/balance.py'),Path('src/reachy_locomotion/primitives.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],score=recovery_score(c['result']),
                                 peak=c['result']['peak_roll_deg'],recovered=c['result']['recovered'],termination=c['result']['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
