"""Refine a rocking gait and its steering using paired physics timesteps.

The first selection failed finer-timestep acceptance. Preserve that evidence and
use its best steered near-miss as a new development anchor, not a held-out test.
"""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_rocking import HIGH,LOW,KEYS,decode,evaluate,read,vector


def paired(job):
    name,g,fb,seconds=job
    checks=[evaluate((name,g,fb,seconds,.001))]
    if checks[0]['metrics']['stable']:
        checks.append(evaluate((name,g,fb,seconds,.0005)))
    ms=[c['metrics'] for c in checks]
    stable=len(ms)==2 and all(m['stable'] for m in ms)
    m=dict(ms[0])
    for k in ['speed_mm_s','stride_mm','bilateral_roll_deg','both_edges_cycle_fraction']:
        m[k]=min(x[k] for x in ms)
    for k in ['cycle_heading_deg','instantaneous_heading_deg']:
        m[k]=max(x[k] for x in ms)
    m['stable']=stable
    m['course_ok']=stable and m['cycle_heading_deg']<=20 and m['stride_mm']>0
    m['style_ok']=m['bilateral_roll_deg']>=6 and m['stride_mm']>=15 and m['both_edges_cycle_fraction']>=.8
    m['feasible']=m['course_ok'] and m['style_ok']
    m['score']=((1000 if m['feasible'] else 0)+m['stride_mm']+min(m['bilateral_roll_deg'],6)
                -3*max(6-m['bilateral_roll_deg'],0)-4*max(m['cycle_heading_deg']-20,0)
                -40*max(.8-m['both_edges_cycle_fraction'],0)) if stable else -1e6
    return dict(id=name,gait=g,feedback=fb,metrics=m,result=checks[0]['result'],checks=checks)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--seed',type=int,default=2026111935)
    p.add_argument('--generations',type=int,default=5)
    p.add_argument('--population',type=int,default=24)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve prior evidence')
    old=read(args.input)
    anchor=next(c for c in old['development'] if c['id']=='6-yaw_only')
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                duration_s=20,physics_timesteps_s=[.001,.0005],history=[],candidates=[],
                note='Previous selection failed; its fine runs are now development evidence. Style gates unchanged.',
                source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('experiments/train_rocking.py'),Path('src/reachy_locomotion/rocking.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    rng=np.random.default_rng(args.seed)
    with ProcessPoolExecutor(max_workers=4) as pool:
        best=next(pool.map(paired,[('anchor',anchor['gait'],anchor['feedback'],20.)]))
        report['candidates'].append(best);save();elite=[best]
        for generation in range(args.generations):
            jobs=[]
            for i in range(args.population):
                base=best if i%2 else elite[i%len(elite)]
                scale=.003 if i%3 else .008
                v=vector(base['gait'])+rng.normal(0,scale*(HIGH-LOW))
                # Keep the proven cycle frequency and waveform family; vary
                # phase, amplitude, translation, braking and steering locally.
                v[KEYS.index('frequency_hz')]=vector(base['gait'])[KEYS.index('frequency_hz')]
                g=decode(v);fb=copy.deepcopy(base['feedback'])
                for key,std,lo,hi in [('kp',.015,.01,.25),('ki',.004,0,.08),
                                      ('yaw_per_roll',.25,-4,-.25),('heading_deadband_deg',.7,1,10)]:
                    fb[key]=float(np.clip(fb[key]+rng.normal(0,std),lo,hi))
                if i==0:g=base['gait'];fb=base['feedback']
                jobs.append((f'g{generation}-{i}',g,fb,20.))
            candidates=list(pool.map(paired,jobs));candidates.sort(key=lambda c:c['metrics']['score'],reverse=True)
            elite=sorted(elite+candidates,key=lambda c:c['metrics']['score'],reverse=True)[:8];best=elite[0]
            report['history'].append(dict(generation=generation,candidates=candidates));report['best']=best;save()
            print(json.dumps(dict(generation=generation,**best['metrics'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
