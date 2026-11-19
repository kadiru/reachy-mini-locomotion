"""Train twice-per-stride yaw steering; preserve the fast head-swing baseline."""
import argparse
import json
import hashlib
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
from reachy_locomotion.heading import HeadingFeedback,HeadingSensor
from select_progress import stress
from train_heading import evaluate,brief


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    base=json.loads(Path('configs/motors-datasheet.json').read_text());configs=[base]+stress(base)
    gait=json.loads(Path('configs/sway-progress-fast.json').read_text());sensor=asdict(HeadingSensor())
    f=asdict(HeadingFeedback(roll_per_correction=0,yaw_harmonic_per_correction=-1,yaw_harmonic_phase_deg=90,
                            startup_ramp_cycles=2,max_correction_deg=6,correction_rate_deg_s=20,kd=.02))
    candidates=[dict(f,kp=kp,ki=ki,filter_tau_s=tau) for kp in [.2,.5,1.,2.] for ki in [.02,.08] for tau in [.08,.25]]
    r=dict(evidence='uncalibrated_simulation',hardware_ready=False,development_seed=591191,configs=configs,sensor=sensor,history=[],
           source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
           [Path(__file__),Path('experiments/train_heading.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        for i,o in enumerate(pool.map(evaluate,[(gait,f,configs,6.5,sensor) for f in candidates])):
            r['history'].append(o);r['best']=max(r['history'],key=lambda o:o['score'])
            path.write_text(json.dumps(r,allow_nan=False)+'\n')
            print(i,'score',round(o['score'],2),'pass',sum(m['feasible'] for m in o['metrics']),
                  'speed',[round(m['speed_mm_s'],1) for m in o['metrics']],flush=True)
    r['complete']=True;r['history'].sort(key=lambda o:o['score'],reverse=True)
    path.write_text(json.dumps(r,allow_nan=False)+'\n');print('BEST',json.dumps(brief(r['best'])),flush=True)
if __name__=='__main__':main()
