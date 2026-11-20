"""Build yaw pulses on bilateral recovery policies; preserve all failed trials."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_balance import evaluate,read,recovery_score

def groups(report):
    result={}
    for c in report['candidates']:result.setdefault(c['id'],[]).append(c)
    return list(result.values())

def signed_pivot(c):
    r=c['result']
    direction=-c['side']
    if r.get('selected_yaw_target_deg'):
        direction=-1 if r['selected_yaw_target_deg']>0 else 1
    return c['pivot_metrics']['passed'] and direction*r['edge_supported_foot_yaw_deg']>=5 and direction*r['final_foot_yaw_deg']>=5

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--mode',choices=['probe','stress'],required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for complete source')
    jobs=[]
    if args.mode=='probe':
        seeds=[cs[0] for cs in groups(source) if len(cs)==10 and all(c['result']['recovered'] for c in cs)]
        for seed in seeds[:3]:
            for i,(yaw,trigger,delay) in enumerate(itertools.product([-40,-25,25,40],[5,10],[.05,.2])):
                g=dict(seed['recipe'],yaw_deg=yaw,yaw_at_roll_deg=trigger,yaw_s=.12,yaw_hold_s=.3,yaw_return_s=.5)
                f=dict(seed['feedback'],yaw_delay_after_catch_s=delay)
                for side in [-1,1]:jobs.append((f"{seed['id']}-yaw-{i}",g,f,side,0,.0005,5.))
    else:
        ranked=sorted(groups(source),key=lambda cs:(sum(signed_pivot(c) for c in cs),min(c['pivot_metrics']['score'] for c in cs)),reverse=True)
        for cs in ranked[:12]:
            seed=cs[0]
            for side,model in itertools.product([-1,1],range(5)):
                jobs.append((seed['id'],seed['recipe'],seed['feedback'],side,model,.0005,5.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            r=c['result'];print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],
                     pivot_pass=signed_pivot(c),edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
