"""Search pure sideways pumping for the tipping boundary, allowing sim falls."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_primitives import evaluate,read,summary

KEYS=['preload_roll_deg','roll_deg','preload_y_mm','y_mm','pitch_deg','x_mm','z_mm','pump_hz','pump_y_phase_deg','pump_sharpness','preload_s']
LOW=np.array([-40,-40,-25,-25,-15,-25,-12,1.,-180,0,.08])
HIGH=np.array([40,40,25,25,15,25,12,5.,180,4,1.])

def score(c):
    r=c['result']
    if not r['model_valid']:return -1000
    sideways_fall=r['fell'] and r['peak_roll_deg']>=max(20,.8*r['peak_tilt_deg'])
    return (1000 if sideways_fall else 0)+r['peak_roll_deg']

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--generations',type=int,default=6);p.add_argument('--population',type=int,default=32)
    p.add_argument('--waist',choices=['off','hold_neutral'],default='hold_neutral')
    p.add_argument('--timestep',type=float,choices=[.001,.0005,.00025],default=.0005)
    p.add_argument('--seed',type=int,default=2026111952)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for complete source')
    unique={}
    for c in sorted(source['candidates'],key=score,reverse=True):unique.setdefault(json.dumps(c['recipe'],sort_keys=True),c)
    anchors=list(unique.values())[:12]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                objective='maximize sideways roll, identifying genuine side falls separately from invalid targets',
                seed=args.seed,initial=[],history=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('src/reachy_locomotion/motors.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    rng=np.random.default_rng(report['seed'])
    with ProcessPoolExecutor(max_workers=4) as pool:
        jobs=[]
        for i,c in enumerate(anchors):
            g=copy.deepcopy(c['recipe']);g['pump_cycles']=12
            jobs.append((f'initial-{i}',g,args.waist,args.timestep,8.))
        anchors=list(pool.map(evaluate,jobs));anchors.sort(key=score,reverse=True)
        report['initial']=anchors;best=anchors[0];save()
        for gen in range(args.generations):
            jobs=[]
            for i in range(args.population):
                a=best if i%2 else anchors[i%len(anchors)]
                v=np.array([a['recipe'][k] for k in KEYS])
                v=np.clip(v+rng.normal(0,(.015 if i%3 else .055)*(HIGH-LOW)),LOW,HIGH)
                g=copy.deepcopy(a['recipe']);g.update(zip(KEYS,v.tolist()))
                if i==0:g=copy.deepcopy(best['recipe'])
                jobs.append((f'g{gen}-{i}',g,args.waist,args.timestep,8.))
            cs=list(pool.map(evaluate,jobs));cs.sort(key=score,reverse=True)
            anchors=sorted(anchors+cs,key=score,reverse=True)[:12];best=anchors[0]
            report['history'].append(dict(generation=gen,candidates=cs));report['best']=best;save()
            print(json.dumps(dict(generation=gen,**summary(best))),flush=True)
            # A measured side fall is a seed for recovery, not a reason to seek
            # an even more violent impact.
            if score(best)>1000:report['stopped_after_side_fall']=True;break
    report['candidates']=anchors;report['completed']=True;save()


if __name__=='__main__':main()
