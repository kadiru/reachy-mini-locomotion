"""Repair promoted failures, then qualify a frozen sustained-walking policy."""
import argparse
import copy
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_faster_balance import read, evaluate as primitive_evaluate
from train_faster_penguin import make_job
from qualify_policy_bank import evaluate

PREFIX='experiments/reports/motor-training-sustained-'
DATE='2026-11-20'

def development():
    r=read('experiments/reports/motor-training-faster-balance-feedback-qualified-2026-11-19.json')
    return copy.deepcopy(r['jobs'][:42])

def run(job):
    start=time.monotonic()
    r=primitive_evaluate(job) if job.get('primitive_only') else evaluate(job)
    r['variant']=job.get('variant','baseline');r['wall_seconds']=time.monotonic()-start
    return r

def batch(jobs,name,workers=4,metadata=None):
    out=Path(f'{PREFIX}{name}-{DATE}.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,
                prior_holdouts_promoted_to_development=True,jobs=jobs,checks=[],**(metadata or {}))
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for r in pool.map(run,jobs):
            report['checks'].append(r);save()
            m=r.get('penguin',{})
            print(json.dumps(dict(variant=r['variant'],case=r['id'],side=r['side'],
                passed=r.get('passed',r['result']['recovered']),termination=r['result']['termination'],
                forward_mm=m.get('forward_mm',1000*r['result']['planar_displacement_m'][0]),
                speed=m.get('speed_mm_s'),heading=m.get('max_completed_cycle_heading_deg'),
                tilt=r['result']['peak_tilt_deg'],seconds=round(r['wall_seconds'],1))),flush=True)
    report['completed']=True;save();return report

def recovery_screen():
    source=next(j for j in development() if j['id']=='unseen-3' and j['side']==-1)
    base=read('configs/penguin-faster-feedback-trained.json')['primary'];variants=[('baseline',copy.deepcopy(base))]
    for trigger in [10,12,14,16]:
        for prediction in [.04,.12]:
            p=copy.deepcopy(base);p['feedback'].update(trigger_deg=trigger,prediction_s=prediction)
            variants.append((f'catch-{trigger}-{prediction}',p))
    for roll in [22,26]:
        for prediction in [0.,.04]:
            p=copy.deepcopy(base);p['feedback'].update(emergency_roll_deg=roll,emergency_prediction_s=prediction)
            variants.append((f'emergency-{roll}-{prediction}',p))
    for after in [.4,.8,1.2]:
        p=copy.deepcopy(base);p['feedback']['recovery_gain_after_catch_s']=after
        variants.append((f'recovery-gains-{after}',p))
    jobs=[]
    for name,p in variants:
        sensors={k:v for k,v in source['policy']['primary']['feedback'].items() if k.startswith('sensor_')}
        jobs.append(dict(id=source['id'],side=-1,variant=name,config=source['config'],
                         primitive=p,sensors=sensors,primitive_only=True))
    return batch(jobs,'recovery-screen')

def sequence_screen():
    base=read('configs/penguin-faster-feedback-trained.json');jobs=[]
    for flag in [False,True]:
        p=copy.deepcopy(base);p['per_side_fallback']=flag
        source=next(j for j in development() if j['id']=='unseen-7' and j['side']==-1)
        j=make_job(source,p,seconds=24);j['variant']=f'per-side-{flag}';jobs.append(j)
    return batch(jobs,'sequence-screen',2)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['recovery','sequence','jobs'])
    parser.add_argument('--jobs');parser.add_argument('--name');parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    if args.mode=='recovery':recovery_screen()
    elif args.mode=='sequence':sequence_screen()
    else:batch(read(args.jobs),args.name,args.workers)
