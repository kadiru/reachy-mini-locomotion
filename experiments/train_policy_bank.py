"""Use observed pivot progress to choose between two learned bounded primitives."""
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

CACHE={}
def policy():
    primary=next(c for c in read('experiments/reports/motor-training-balance-lateral-stress-2026-11-19.json')['candidates'] if c['id']=='lateral-4')
    fallback=next(c for c in read('experiments/reports/motor-training-balance-launch-stress-2026-11-19.json')['candidates'] if c['id']=='g3-11')
    def compact(c):return dict(recipe=c['recipe'],feedback=dict(c['feedback'],return_x_after_catch_s=1.5,return_x_duration_s=.6))
    return dict(primary=compact(primary),fallback=compact(fallback),target_step_turn_deg=15,max_repeats=3,switch_below_deg=3.)

def evaluate(job):
    model,side,pivot_only=job;p=policy();config=copy.deepcopy(models()[model]);config['timestep_s']=.0005
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];g=p['primary']['recipe'];f=p['primary']['feedback']
    ctl=PenguinController(g,f,cycles=1,min_step_s=3.,start_side=side,checked_ik=sim.checked_ik,
                         target_step_turn_deg=p['target_step_turn_deg'],max_repeats=p['max_repeats'],pivot_only=pivot_only,
                         heading_noise_deg=.1,heading_seed=42,fallback_policy=p['fallback'],switch_below_deg=p['switch_below_deg'])
    r=sim.run_primitive(mirrored(g,side),seconds=15 if pivot_only else 28,waist_mode='hold_neutral',controller=ctl,tilt_limit_deg=35)
    trace=r.pop('trace');r.pop('terminal_state');s=ctl.report();pm=pivot_metrics(r)
    if not pivot_only:s.update(metrics(r,trace,s))
    passed=(s['completed'] and pm['passed'] and -side*r['final_foot_yaw_deg']>=5 and -side*r['edge_supported_foot_yaw_deg']>=5) if pivot_only else s['passed']
    return dict(model_id=model,side=side,pivot_only=pivot_only,result=r,penguin=s,pivot_metrics=pm,passed=passed)

def main():
    out=Path('experiments/reports/motor-training-policy-bank-development-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,frozen_policy=policy(),checks=[])
    out.write_text(json.dumps(report)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,list(itertools.product(range(5),[-1,1],[True,False]))):
            report['checks'].append(c);out.write_text(json.dumps(report,allow_nan=False)+'\n')
            r=c['result'];s=c['penguin'];print(json.dumps(dict(model=c['model_id'],side=c['side'],pivot=c['pivot_only'],passed=c['passed'],
                 complete=s['completed'],bursts=len(s['bursts']),fallback=s['fallback_activated'],edge=r['edge_supported_foot_yaw_deg'],
                 yaw=r['final_foot_yaw_deg'],forward_mm=1000*r['planar_displacement_m'][0],termination=r['termination'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report,allow_nan=False)+'\n')

if __name__=='__main__':main()
