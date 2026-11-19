"""Train bounded head-only heading feedback on the previously failed stress set."""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator
from reachy_locomotion.heading import HeadingFeedback, HeadingSensor
from select_progress import stress
from train_progress import metrics

CACHE={}

def evaluate(payload):
    gait,feedback,configs,seconds,sensor=payload
    results=[]
    for c in configs:
        key=json.dumps(c,sort_keys=True)
        if key not in CACHE:CACHE[key]=SwaySimulator('experiments/upstream-reachy-mini',c)
        g=Gait(**gait)
        results.append(CACHE[key].run(g,cycles=math.ceil(seconds/(round(50/g.frequency_hz)/50)),
                                      warmup_cycles=3,feedback=feedback,sensor=sensor))
    mm=[metrics(r) for r in results]
    violations=[m['violation']+max(0.,np.degrees(r['max_heading_from_startup_rad'])-10.) for m,r in zip(mm,results)]
    speeds=[m['speed_mm_s'] for m in mm]
    feasible=all(m['feasible'] for m in mm) and max(violations)<1e-9
    # Keep nominal speed, while emphasizing the worst domains. No smoothness reward.
    speed=.35*min(speeds)+.35*np.mean(speeds)+.3*speeds[0]
    retained_speed = speeds[0] >= 14.
    score=(1000+speed) if feasible and retained_speed else speed-3*max(violations)-np.mean(violations)-4*max(0.,14.-speeds[0])
    return dict(gait=gait,feedback=feedback,score=float(score),feasible_all=bool(feasible),
                nominal_speed_target_mm_s=14.,speed_target_met=bool(retained_speed),metrics=mm,results=results)

def brief(o):
    return dict(score=round(o['score'],3),feasible=o['feasible_all'],feedback=o['feedback'],
                speeds=[round(m['speed_mm_s'],2) for m in o['metrics']],
                heading=[round(np.degrees(r['max_heading_from_startup_rad']),2) for r in o['results']])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['grid','averaged-grid','yaw-grid','cem'],default='grid');p.add_argument('--seed-report')
    p.add_argument('--generations',type=int,default=8);p.add_argument('--population',type=int,default=24)
    a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new report')
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    gait=json.loads(Path('configs/sway-progress-fast.json').read_text())
    configs=[base]+stress(base);sensor=asdict(HeadingSensor())
    receipt=dict(evidence='uncalibrated_simulation',hardware_ready=False,development_seed=591191,
                 formerly_heldout_now_training=True,sensor=sensor,configs=configs,history=[],args=vars(a),
                 source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/heading.py'),Path('src/reachy_locomotion/sway.py')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        if a.mode in ('grid','averaged-grid','yaw-grid'):
            candidates=[dict(asdict(HeadingFeedback()),kp=kp,ki=ki,filter_tau_s=tau)
                        for kp in [.04,.08,.16,.32] for ki in [.01,.04,.1] for tau in [.15,.35]]
            if a.mode=='averaged-grid':
                candidates=[dict(asdict(HeadingFeedback()),kp=kp,ki=.025,kd=kd,filter_tau_s=.04,
                                 cycle_average=True,startup_ramp_cycles=3.,max_correction_deg=2.,correction_rate_deg_s=4.)
                            for kp in [.04,.1,.2,.4] for kd in [.02,.06,.12,.24]]
            if a.mode=='yaw-grid':
                candidates=[dict(asdict(HeadingFeedback()),kp=kp,ki=ki,kd=kd,filter_tau_s=.08,
                                 roll_per_correction=0.,yaw_per_roll=1.,startup_ramp_cycles=2.,
                                 max_correction_deg=5.,correction_rate_deg_s=20.)
                            for kp in [.2,.5,1.] for ki in [.03,.1] for kd in [0.,.05]]
            for i,o in enumerate(pool.map(evaluate,[(gait,f,configs,6.5,sensor) for f in candidates])):
                receipt['history'].append(o);path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
                print(i,json.dumps(brief(o)),flush=True)
        else:
            old=json.loads(Path(a.seed_report).read_text());archive=old['history'];archive.sort(key=lambda x:x['score'],reverse=True)
            keys=['kp','ki','kd','filter_tau_s','max_correction_deg','correction_rate_deg_s','yaw_per_roll']
            low=np.array([.015,0,0,.06,1.,1.,-1.5]);high=np.array([.5,.3,.12,.5,5.,20.,1.5])
            rng=np.random.default_rng(2026111904)
            def vector(o):return np.array([o['feedback'][k] for k in keys])
            best=archive[0];mean=vector(best);std=.15*(high-low)
            for gen in range(a.generations):
                samples=np.clip(rng.normal(mean,std,size=(a.population,len(keys))),low,high)
                samples[0]=vector(best)
                for i,o in enumerate(archive[:3]):samples[i+1]=vector(o)
                extras={k:v for k,v in best['feedback'].items() if k not in keys}
                out=list(pool.map(evaluate,[(gait,dict(extras,**dict(zip(keys,row.tolist()))),configs,8.,sensor) for row in samples]))
                out.sort(key=lambda o:o['score'],reverse=True)
                # All comparisons within this stage use the same duration.
                best=out[0];archive=out
                elite=np.array([vector(o) for o in out[:5]])
                mean=.2*mean+.8*elite.mean(axis=0);std=np.maximum(.3*std+.7*elite.std(axis=0),.02*(high-low))
                receipt['history'].extend(out);receipt['best']=best
                path.write_text(json.dumps(receipt,allow_nan=False)+'\n');print(gen,json.dumps(brief(best)),flush=True)
        receipt['history'].sort(key=lambda o:o['score'],reverse=True);receipt['best']=receipt['history'][0]
    receipt['complete']=True;path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    print('BEST',json.dumps(brief(receipt['best'])),flush=True)
if __name__=='__main__':main()
