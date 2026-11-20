"""Train side-specific physical launch/capture into the learned edge regulator."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from train_edge_balance import evaluate,read

PARAMS={
    ('recipe','catch_at_deg'):(6,14,1.1),('recipe','catch_s'):(.08,.28,.025),
    ('recipe','catch_hold_s'):(.08,.4,.045),('recipe','x_mm'):(0,16,2),
    ('recipe','pitch_deg'):(-8,2,1.2),
    ('feedback','launch_return_x_after_s'):(0,.5,.08),('feedback','launch_return_x_duration_s'):(.15,.8,.08),
    ('feedback','trigger_deg'):(23,32,2),('feedback','capture_blend_s'):(.02,.3,.045),
}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('policy');p.add_argument('--output',required=True)
    p.add_argument('--side',type=int,choices=[-1,1],required=True);p.add_argument('--generations',type=int,default=8);p.add_argument('--population',type=int,default=32)
    p.add_argument('--seed',type=int,default=2026112017);p.add_argument('--sigma',type=float,default=1);p.add_argument('--timestep',type=float,default=.00025)
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    policy=read(a.policy);keys=list(PARAMS);bounds=np.array(list(PARAMS.values()));lo,hi=bounds[:,0],bounds[:,1]
    mu=np.array([policy[group][key] for group,key in keys]);sigma=bounds[:,2]*a.sigma;rng=np.random.default_rng(a.seed);best=None
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,args=vars(a),completed=False,candidates=[],
        source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),Path('src/reachy_locomotion/edge_linear.py'),Path('src/reachy_locomotion/edge_balance.py'),Path('src/reachy_locomotion/primitives.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=6) as pool:
        for gen in range(a.generations):
            xs=np.clip(rng.normal(mu,sigma,(a.population,len(keys))),lo,hi);xs[0]=mu
            if best:xs[1]=[best['policy'][g][k] for g,k in keys]
            jobs=[]
            for i,x in enumerate(xs):
                pol=copy.deepcopy(policy)
                for (g,k),v in zip(keys,x):pol[g][k]=float(v)
                jobs.append(dict(id=f'capture-g{gen}-{i}',policy=pol,side=a.side,seconds=9,dt=a.timestep,early_stop_flat=True))
            cs=[]
            for c in pool.map(evaluate,jobs):cs.append(c);report['candidates'].append(c);save()
            cs.sort(key=lambda c:c['score'],reverse=True)
            if best is None or cs[0]['score']>best['score']:best=cs[0]
            elites=cs[:max(4,a.population//5)];values=np.array([[c['policy'][g][k] for g,k in keys] for c in elites])
            mu=.2*mu+.8*values.mean(axis=0);sigma=np.maximum(.25*sigma+.75*values.std(axis=0),.006*(hi-lo))
            report['best']=best;save();print(json.dumps(dict(generation=gen,best=best['id'],score=best['score'],termination=best['result']['termination'],metrics=best['metrics'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
