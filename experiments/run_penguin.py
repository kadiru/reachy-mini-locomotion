"""Replay the frozen bilateral controller, optionally saving every physical state."""
import argparse
import json
from pathlib import Path
from train_balance import models
from qualify_policy_bank import evaluate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('policy');p.add_argument('--output',required=True)
    p.add_argument('--record');p.add_argument('--cycles',type=int,default=2)
    p.add_argument('--seconds',type=float,default=32)
    p.add_argument('--side',type=int,choices=[-1,1],default=1)
    p.add_argument('--model',type=int,choices=range(5),default=0)
    p.add_argument('--timestep',type=float,default=.00025)
    p.add_argument('--pivot-only',action='store_true')
    args=p.parse_args();out=Path(args.output)
    if out.exists() or (args.record and Path(args.record).exists()):p.error('Preserve previous evidence')
    config=models()[args.model];config['timestep_s']=args.timestep
    job=dict(id='replay',policy=json.loads(Path(args.policy).read_text()),config=config,side=args.side,
             pivot_only=args.pivot_only,cycles=args.cycles,seconds=args.seconds,record=args.record)
    r=evaluate(job);out.write_text(json.dumps(dict(job=job,check=r),allow_nan=False)+'\n')
    print(json.dumps(dict(passed=r['passed'],termination=r['result']['termination'],penguin=r['penguin'])))

if __name__=='__main__':main()
