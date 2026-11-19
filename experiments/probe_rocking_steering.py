"""Test bounded, longer-horizon steering without changing motor limits."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from refine_rocking_feedback import paired
from train_rocking import read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    s=read(args.input);anchor=next(c for c in s['development'] if c['id']=='0-learned')
    jobs=[];g=anchor['gait']
    for yaw in [-2,2]:
        for cap in [.35,.7,1.4]:
            fb=dict(anchor['feedback'],max_correction_deg=cap,kp=.08,ki=.005,yaw_per_roll=yaw)
            jobs.append((f'yaw-{yaw}-cap-{cap}',g,fb,40.))
    for mode in ['endpoint','average']:
        for phase in [-.04,-.015,.015,.04]:
            fb=dict(anchor['feedback'],max_correction_deg=1.5,kp=.12,ki=.005,yaw_per_roll=0,
                    phase_skew_per_correction=phase,heading_deadband_deg=3,
                    cycle_endpoint=mode=='endpoint',cycle_average=mode=='average',cycle_update=mode=='average')
            jobs.append((f'{mode}-phase-{phase}',g,fb,40.))
        for roll in [-.3,.3]:
            fb=dict(anchor['feedback'],max_correction_deg=1,kp=.12,ki=.005,yaw_per_roll=0,
                    roll_per_correction=roll,heading_deadband_deg=3,
                    cycle_endpoint=mode=='endpoint',cycle_average=mode=='average',cycle_update=mode=='average')
            jobs.append((f'{mode}-roll-{roll}',g,fb,40.))
    for mode in ['endpoint','average']:
        for yaw in [-2,2]:
            fb=dict(anchor['feedback'],max_correction_deg=1,kp=.04,ki=0,yaw_per_roll=yaw,
                    heading_deadband_deg=2,cycle_endpoint=mode=='endpoint',cycle_average=mode=='average',cycle_update=mode=='average')
            jobs.append((f'{mode}-low-p-yaw-{yaw}',g,fb,40.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                candidates=[],duration_s=40,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('experiments/refine_rocking_feedback.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(paired,jobs):
            report['candidates'].append(c);save();print(json.dumps(dict(id=c['id'],**c['metrics'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
