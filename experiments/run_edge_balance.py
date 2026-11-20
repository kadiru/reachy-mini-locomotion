"""Replay a frozen edge-balance skill in simulation; never connects to a robot."""
import argparse
import json
from pathlib import Path
from train_edge_balance import evaluate,read


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('policy');p.add_argument('--side',choices=['left','right'],required=True)
    p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=20);p.add_argument('--timestep',type=float,default=.000125)
    p.add_argument('--record',action='store_true');p.add_argument('--config',default='configs/motors-speed-trained.json')
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    source=read(a.policy);policy=source['sides'][a.side] if 'sides' in source else source
    out.parent.mkdir(parents=True,exist_ok=True)
    job=dict(id='replay-'+a.side,policy=policy,side=-1 if a.side=='left' else 1,seconds=a.seconds,dt=a.timestep,config=read(a.config))
    if a.record:job['record']=str(out)
    result=evaluate(job)
    if not a.record:out.write_text(json.dumps(result,allow_nan=False)+'\n')
    print(json.dumps(dict(termination=result['result']['termination'],metrics=result['metrics']),indent=2))


if __name__=='__main__':main()
