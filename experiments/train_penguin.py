"""Compose edge primitives into alternating steps; no physical state resets."""
import argparse
import copy
import itertools
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.penguin import PenguinController
from reachy_locomotion.balance import mirrored
from reachy_locomotion.primitives import PrimitiveSimulator
from train_balance import read,models
from train_balance_pivot import groups,signed_pivot

CACHE={}
def evaluate(job):
    label,g,f,model_id,dt,cycles,min_step,side=job
    config=copy.deepcopy(models()[model_id]);config['timestep_s']=dt
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];controller=PenguinController(g,f,cycles,min_step_s=min_step,start_side=side,checked_ik=sim.checked_ik)
    seconds=round((2*cycles*(min_step+.8)+2)/.02)*.02
    r=sim.run_primitive(mirrored(g,side),seconds=seconds,waist_mode='hold_neutral',controller=controller,tilt_limit_deg=35)
    summary=controller.report();trace=r.pop('trace');r.pop('terminal_state')
    summary.update(metrics(r,trace,summary))
    return dict(id=label,recipe=g,feedback=f,model_id=model_id,timestep_s=dt,start_side=side,min_step_s=min_step,cycles=cycles,result=r,penguin=summary)

def metrics(r,trace,summary):
    pairs=[]
    for i in range(0,len(summary['steps'])-1,2):
        a=summary['steps'][i]['start_s'];b=summary['steps'][i+1]['end_s']
        rows=[x for x in trace if a-1e-8<=x['time_s']<=b+1e-8]
        if not rows:continue
        start,end=rows[0],rows[-1]
        dx=(end['foot_xyz_m'][0]-start['foot_xyz_m'][0])*1000
        rolls=[x['roll_deg'] for x in rows]
        pairs.append(dict(forward_mm=dx,heading_deg=end['foot_yaw_deg'],bilateral_roll_deg=min(max(rolls),-min(rolls)),duration_s=b-a))
    x=r['planar_displacement_m'][0]*1000
    duration=summary['steps'][-1]['end_s'] if summary['steps'] else r['requested_seconds']
    stride=float(np.mean([p['forward_mm'] for p in pairs])) if pairs else 0.
    bilateral=min((p['bilateral_roll_deg'] for p in pairs),default=0.)
    stable=r['completed'] and r['settled'] and summary['completed']
    course=stable and summary['max_completed_cycle_heading_deg']<=20
    passed=course and stride>=15 and bilateral>=6 and all(p['forward_mm']>0 for p in pairs)
    return dict(passed=passed,stable=stable,course_ok=course,mean_stride_mm=stride,min_bilateral_roll_deg=bilateral,
                forward_mm=x,speed_mm_s=x/max(duration,.02),full_trial_speed_mm_s=x/r['requested_seconds'],pairs=pairs,
                score=(1000 if passed else 0)+(100 if stable else -1000)+x-3*max(0,summary['max_completed_cycle_heading_deg']-20))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['probe','stress'],required=True)
    p.add_argument('--neutral-return',action='store_true')
    p.add_argument('--recovery-gate',help='Require all development primitive trials to recover before composing')
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for completed source')
    jobs=[]
    if args.mode=='probe':
        ranked=sorted(groups(source),key=lambda cs:(sum(signed_pivot(c) for c in cs),min(c['pivot_metrics']['score'] for c in cs)),reverse=True)
        unique={}
        for cs in ranked:
            c=cs[0];unique.setdefault(json.dumps([c['recipe'],c['feedback']],sort_keys=True),c)
        for c in list(unique.values())[:6]:
            f=dict(c['feedback'])
            if args.neutral_return:f.update(return_x_after_catch_s=1.5,return_x_duration_s=.6)
            for min_step,side in itertools.product([3.,3.5] if args.neutral_return else [2.2,2.6,3.],[1,-1]):
                jobs.append((c['id'],c['recipe'],f,0,.0005 if args.neutral_return else .001,2,min_step,side))
    else:
        ranked=sorted(source['candidates'],key=lambda c:c['penguin']['score'],reverse=True)
        if args.recovery_gate:
            allowed={cs[0]['id'] for cs in groups(read(args.recovery_gate)) if len(cs)==10 and all(c['result']['recovered'] for c in cs)}
            unique={}
            for c in ranked:
                if c['id'] in allowed:unique.setdefault(c['id'],c)
            ranked=list(unique.values())
        for c in ranked[:4]:
            for model in range(5):
                jobs.append((c['id'],c['recipe'],c['feedback'],model,.0005,3,c['min_step_s'],c['start_side']))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                gates=dict(mean_stride_mm=15,min_bilateral_roll_deg=6,max_cycle_heading_deg=20,all_steps_complete=True,final_settled=True),candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            print(json.dumps(dict(id=c['id'],model=c['model_id'],side=c['start_side'],step_s=c['min_step_s'],termination=c['result']['termination'],**c['penguin'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
