"""Repeat settled edge primitives until a delayed heading target is reached."""
import argparse
import copy
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from reachy_locomotion.balance import mirrored
from reachy_locomotion.penguin import PenguinController
from reachy_locomotion.primitives import PrimitiveSimulator
from train_balance import read,models
from qualify_primitives import pivot_metrics
from train_penguin import metrics
from train_balance_pivot import groups,signed_pivot

CACHE={}
def evaluate(job):
    label,g,f,model,side,pivot_only,target,dt=job
    config=copy.deepcopy(models()[model]);config['timestep_s']=dt;key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];ctl=PenguinController(g,f,cycles=1,min_step_s=3.,start_side=side,checked_ik=sim.checked_ik,
                target_step_turn_deg=target,max_repeats=3,pivot_only=pivot_only,heading_noise_deg=.1,heading_seed=42)
    seconds=15 if pivot_only else 28
    r=sim.run_primitive(mirrored(g,side),seconds=seconds,waist_mode='hold_neutral',controller=ctl,tilt_limit_deg=35)
    trace=r.pop('trace');r.pop('terminal_state');summary=ctl.report()
    if not pivot_only:summary.update(metrics(r,trace,summary))
    return dict(id=label,recipe=g,feedback=f,model_id=model,side=side,result=r,penguin=summary,pivot_metrics=pivot_metrics(r))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--walking',action='store_true');args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for source')
    ranked=sorted(groups(source),key=lambda cs:(sum(c['result']['recovered'] for c in cs),sum(signed_pivot(c) for c in cs),min(c['pivot_metrics']['score'] for c in cs)),reverse=True)
    jobs=[]
    for cs in ranked[:2]:
        c=cs[0];f=dict(c['feedback'],return_x_after_catch_s=1.5,return_x_duration_s=.6)
        for model,side in itertools.product(range(5),[-1,1]):
            jobs.append((c['id'],c['recipe'],f,model,side,not args.walking,15,.0005))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                additional_sensor='20ms-delayed simulated base heading, 0.1 degree noise; hardware estimator uncalibrated',candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result'];s=c['penguin']
            print(json.dumps(dict(id=c['id'],model=c['model_id'],side=c['side'],recovered=r['recovered'],goal_complete=s['completed'],
                  pivot_pass=c['pivot_metrics']['passed'],walk_pass=s.get('passed'),bursts=len(s['bursts']),edge=r['edge_supported_foot_yaw_deg'],
                  yaw=r['final_foot_yaw_deg'],forward_mm=1000*r['planar_displacement_m'][0],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
