"""Separate steering and single-coordinate gait changes after paired search."""
import argparse
import copy
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
    if out.exists():p.error('Preserve prior evidence')
    source=read(args.input)
    if not source['completed']:p.error('Wait for paired search')
    best=source['best'];g=best['gait'];fb=best['feedback']
    jobs=[]
    for key,values in [('kp',[.02,.04,.06,.1,.12]),('ki',[0,.005,.03]),
                       ('yaw_per_roll',[-.5,-1,1,2]),('heading_deadband_deg',[2,8,12])]:
        for value in values:
            variant=dict(fb);variant[key]=value
            jobs.append((f'{key}-{value}',g,variant,20.))
    for key,values in [('frequency_hz',[50/17,50/18,50/15]),
                       ('roll_deg',[g['roll_deg']-.3,g['roll_deg']+.3]),
                       ('yaw_phase_deg',[g['yaw_phase_deg']-.5,g['yaw_phase_deg']+.5]),
                       ('pitch_bias_deg',[g['pitch_bias_deg']-.3,g['pitch_bias_deg']+.3]),
                       ('stroke_brake_fraction',[.025,.05,.08])]:
        for value in values:
            variant=dict(g);variant[key]=value
            jobs.append((f'{key}-{value}',variant,fb,20.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                candidates=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('experiments/refine_rocking_feedback.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(paired,jobs):
            report['candidates'].append(c);save();print(json.dumps(dict(id=c['id'],**c['metrics'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
