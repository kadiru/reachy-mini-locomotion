"""Align catch and yaw with signed rocking phase across model distributions."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    source=read('experiments/reports/motor-training-balance-stress-2026-11-19.json')
    seeds={c['id']:c for c in source['candidates'] if c['id'] in ['pivot-5','pivot-17']}
    jobs=[]
    for family,seed in seeds.items():
        for i,(catch_sign,yaw_pred,yaw) in enumerate(itertools.product([-1,1],[0,.08],[0,40,60])):
            g=dict(seed['recipe'],yaw_deg=yaw,yaw_at_roll_deg=5,yaw_s=.12,yaw_hold_s=.2,yaw_return_s=1.)
            f=dict(seed['feedback'],catch_direction=catch_sign,yaw_direction=-catch_sign,yaw_prediction_s=yaw_pred,
                   yaw_delay_after_catch_s=.02,state_dependent_yaw=True,workspace_projection=True)
            for side,model in itertools.product([-1,1],[0,2]):jobs.append((f'{family}-signed-{i}',g,f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
