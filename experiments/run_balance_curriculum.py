"""Replay a frozen hold, return cycle or two physical ball impacts in simulation."""
import argparse
import copy
import json
from pathlib import Path

from train_balance_robustness import evaluate,read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('policy');p.add_argument('--edge',choices=['left','right','front','back'],required=True)
    p.add_argument('--mode',choices=['hold','cycle','impacts'],default='hold')
    p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=32)
    p.add_argument('--timestep',type=float,default=.000125);p.add_argument('--record',action='store_true')
    p.add_argument('--raised-start-deg',type=float,help='Explicit artificial initial angle; never a natural launch')
    p.add_argument('--outward-rate-deg-s',type=float,default=0)
    p.add_argument('--capture-policy',action='store_true');p.add_argument('--config',default='configs/motors-speed-trained.json')
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve earlier evidence')
    source=read(a.policy);bank=source['capture_policies'] if a.capture_policy else source['edges']
    if a.edge not in bank:p.error('No frozen policy for this edge/mode')
    policy=copy.deepcopy(bank[a.edge]);job=dict(id='replay-'+a.edge,edge=a.edge,policy=policy,seconds=a.seconds,dt=a.timestep,config=read(a.config))
    if a.raised_start_deg is not None:job['initial_angle_deg']=a.raised_start_deg
    job['outward_rate_deg_s']=a.outward_rate_deg_s
    if a.mode in ('cycle','impacts'):policy['return']=copy.deepcopy(source['controlled_return'])
    if a.mode=='impacts':
        if a.edge not in ('left','right'):p.error('The frozen rolling-ball curriculum is for left/right balance')
        policy['return']['hold_s']=12
        job['balls']=[dict(source['rolling_ball'],launch_s=8,from_side=1),dict(source['rolling_ball'],launch_s=12,from_side=-1)]
    if a.record:job['record']=str(out)
    result=evaluate(job)
    if not a.record:
        out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,allow_nan=False)+'\n')
    print(json.dumps(dict(termination=result['result']['termination'],metrics=result['metrics'],curriculum=result['result']['balance_curriculum']),indent=2))


if __name__=='__main__':main()
