"""Retrain head-return timing on promoted failures before composing faster steps."""
import argparse
import copy
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from reachy_locomotion.balance import BalanceController,mirrored
from reachy_locomotion.primitives import PrimitiveSimulator
from qualify_primitives import pivot_metrics

CACHE={}
def read(path):return json.loads(Path(path).read_text())

def cases():
    previous=read('experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json')
    return [copy.deepcopy(j) for j in previous['frozen_jobs'] if j['pivot_only'] and not j['id'].startswith('nominal-')]

def evaluate(job):
    config=job['config'];key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];p=job['primitive'];g=mirrored(p['recipe'],job['side'])
    f=dict(p['feedback']);f.update(job['sensors'])
    ctl=BalanceController(g,f,checked_ik=sim.checked_ik)
    r=sim.run_primitive(g,seconds=5,waist_mode='hold_neutral',controller=ctl,tilt_limit_deg=35)
    trace=r.pop('trace');r.pop('terminal_state')
    return dict(id=job['id'],variant=job['variant'],side=job['side'],primitive=p,result=r,
                pivot_metrics=pivot_metrics(r),signed_turn_deg=-job['side']*r['final_foot_yaw_deg'],
                first_settled_after_catch_s=next((x['time_s'] for x in trace if ctl.catch_trigger is not None and
                    x['time_s']>ctl.catch_trigger+1. and abs(x['roll_deg'])<2 and x['base_angular_speed_rad_s']<.2),None))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    parser.add_argument('--mode',choices=['timing','stress'],default='timing');parser.add_argument('--input')
    parser.add_argument('--top',type=int,default=2)
    parser.add_argument('--timestep',type=float,default=.0005)
    parser.add_argument('--variants',help='Comma-separated already screened candidates')
    args=parser.parse_args();out=Path(args.output)
    if out.exists():parser.error('Preserve previous evidence')
    bank=read('configs/penguin-feedback-experimental.json');jobs=[];variants=[]
    if args.mode=='timing':
        for after,duration,yaw_return in itertools.product([.2,.6,1.],[.2,.4],[.4,.7,1.]):
            p=copy.deepcopy(bank['primary']);p['feedback'].update(return_x_after_catch_s=after,return_x_duration_s=duration)
            p['recipe']['yaw_return_s']=yaw_return
            variants.append((f'x{after}-{duration}-yaw{yaw_return}',p))
        variants.insert(0,('baseline',bank['primary']))
        selected=[j for j in cases() if j['id'] in ('development-0','fresh-3','fresh-7')]
    else:
        source=read(args.input);assert source['completed']
        groups={}
        for c in source['checks']:groups.setdefault(c['variant'],[]).append(c)
        def score(cs):
            return (sum(c['result']['recovered'] for c in cs),min(c['signed_turn_deg'] for c in cs),
                    min(c['result']['planar_displacement_m'][0] for c in cs))
        ranked=sorted(groups.items(),key=lambda kv:score(kv[1]),reverse=True)
        chosen=ranked[:args.top] if not args.variants else [(n,groups[n]) for n in args.variants.split(',')]
        variants=[(name,cs[0]['primitive']) for name,cs in chosen];selected=cases()
    for name,p in variants:
        for original in selected:
            c=copy.deepcopy(original['config']);c['timestep_s']=args.timestep
            sensors={k:v for k,v in original['policy']['primary']['feedback'].items() if k.startswith('sensor_')}
            jobs.append(dict(id=original['id'],variant=name,side=original['side'],config=c,primitive=p,sensors=sensors))
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,args=vars(args),
                prior_fresh_cases_are_now_development=True,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save()
            print(json.dumps(dict(id=c['id'],side=c['side'],variant=c['variant'],recovered=c['result']['recovered'],
                  termination=c['result']['termination'],turn=c['signed_turn_deg'],forward_mm=1000*c['result']['planar_displacement_m'][0])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
