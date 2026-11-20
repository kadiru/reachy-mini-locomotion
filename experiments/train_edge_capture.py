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
    p.add_argument('--foreaft',action='store_true');p.add_argument('--seconds',type=float,default=9);p.add_argument('--workers',type=int,default=6)
    p.add_argument('--stop-on-hold',action='store_true',help='Finish after a generation produces a qualified five-second hold')
    p.add_argument('--pump',action='store_true',help='Search the repeated nodding launch instead of the first-catch recipe')
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    params=dict(PARAMS);evaluator=evaluate
    if a.foreaft:
        from train_foreaft_balance import evaluate as foreaft_evaluate
        evaluator=foreaft_evaluate;params[('recipe','x_mm')]=(-12,12,2);params[('recipe','pitch_deg')]=(-8,8,1.2)
    if a.pump:
        params={('recipe','preload_roll_deg'):(-40,-20,3),('recipe','roll_deg'):(20,40,3),
                ('recipe','preload_y_mm'):(8,25,3),('recipe','y_mm'):(-25,-8,3),
                ('recipe','z_mm'):(-15,5,2),('recipe','pump_hz'):(2.2,3.5,.18),
                ('recipe','pump_y_phase_deg'):(-110,30,15),('recipe','pump_sharpness'):(0,2,.2),
                ('feedback','trigger_deg'):(18,32,2),('feedback','prediction_s'):(0,.15,.025),
                ('feedback','capture_blend_s'):(.02,.3,.04)}
    policy=read(a.policy);keys=list(params);bounds=np.array(list(params.values()));lo,hi=bounds[:,0],bounds[:,1]
    mu=np.array([policy[group][key] for group,key in keys]);sigma=bounds[:,2]*a.sigma;rng=np.random.default_rng(a.seed);best=None
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,args=vars(a),completed=False,candidates=[],
        source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),Path('src/reachy_locomotion/edge_linear.py'),Path('src/reachy_locomotion/edge_balance.py'),Path('src/reachy_locomotion/primitives.py')]})
    if a.foreaft:
        for x in [Path('src/reachy_locomotion/foreaft_balance.py'),Path('experiments/train_foreaft_balance.py')]:report['source_sha256'][str(x)]=hashlib.sha256(x.read_bytes()).hexdigest()
    def save():
        tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(report,allow_nan=False)+'\n');tmp.replace(out)
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for gen in range(a.generations):
            xs=np.clip(rng.normal(mu,sigma,(a.population,len(keys))),lo,hi);xs[0]=mu
            if best:xs[1]=[best['policy'][g][k] for g,k in keys]
            jobs=[]
            for i,x in enumerate(xs):
                pol=copy.deepcopy(policy)
                for (g,k),v in zip(keys,x):pol[g][k]=float(v)
                jobs.append(dict(id=f'capture-g{gen}-{i}',policy=pol,side=a.side,seconds=a.seconds,dt=a.timestep,early_stop_flat=True))
            cs=[]
            for c in pool.map(evaluator,jobs):cs.append(c);report['candidates'].append(c);save()
            cs.sort(key=lambda c:c['score'],reverse=True)
            if best is None or cs[0]['score']>best['score']:best=cs[0]
            elites=cs[:max(4,a.population//5)];values=np.array([[c['policy'][g][k] for g,k in keys] for c in elites])
            mu=.2*mu+.8*values.mean(axis=0);sigma=np.maximum(.25*sigma+.75*values.std(axis=0),.006*(hi-lo))
            report['best']=best;save();print(json.dumps(dict(generation=gen,best=best['id'],score=best['score'],termination=best['result']['termination'],metrics=best['metrics'])),flush=True)
            if a.stop_on_hold and best['metrics']['final_five_seconds_pass']:
                report['stopped_after_qualified_hold']=True;break
    report['completed']=True;save()

if __name__=='__main__':main()
