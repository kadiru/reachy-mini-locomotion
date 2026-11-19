"""Test yaw/brake/hold timing and preserve a sway-only pivot comparison."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_primitives import evaluate,read,summary
from qualify_primitives import pivot_metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catches',required=True);p.add_argument('--pivot-search',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    catches=read(args.catches);search=read(args.pivot_search)
    if not catches['completed'] or not search['completed']:p.error('Wait for completed sources')
    natural=max(catches['candidates'],key=lambda c:min(pivot_metrics(c['result'])['score'],pivot_metrics(c['coarser_result'])['score']))
    prior=max([c for c in search['candidates'] if c['result']['recovered']],key=lambda c:pivot_metrics(c['result'])['score'])
    seeds=[natural,prior];jobs=[]
    for i,c in enumerate(seeds):
        g=dict(c['recipe'],yaw_deg=0);jobs.append((f'{i}-no-yaw',g,'hold_neutral',.0005,8.))
        for yaw,trigger in itertools.product([-60,-40,-20,20,40,60],[4,8,12,18]):
            g=dict(c['recipe'],yaw_deg=yaw,yaw_at_roll_deg=trigger,yaw_s=.1,yaw_hold_s=.4,yaw_return_s=.6)
            jobs.append((f'{i}-yaw{yaw}-at{trigger}',g,'hold_neutral',.0005,8.))
    for yaw,hold,ret,trigger in itertools.product([-50,-60],[.05,.3,.6,.8],[.3,.7],[12,14]):
        g=dict(prior['recipe'],yaw_deg=yaw,yaw_hold_s=hold,yaw_return_s=ret,yaw_at_roll_deg=trigger)
        jobs.append((f'hold-y{yaw}-h{hold}-r{ret}-at{trigger}',g,'hold_neutral',.0005,8.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                selected_seeds=seeds,candidates=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('experiments/qualify_primitives.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            c['pivot_metrics']=pivot_metrics(c['result']);report['candidates'].append(c);save()
            print(json.dumps(dict(**summary(c),pivot=c['pivot_metrics'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
