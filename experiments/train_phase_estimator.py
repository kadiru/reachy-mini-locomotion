"""Tune feedback using latency-compensated separation of rocking and heading drift."""
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
    f=asdict(HeadingFeedback(phase_fit=True,filter_tau_s=.04,startup_ramp_cycles=2.,max_correction_deg=3.,correction_rate_deg_s=8.))
    candidates=[dict(f,kp=kp,ki=ki,kd=kd) for kp in [.04,.1,.2,.4] for ki in [.01,.05] for kd in [0.,.08]]
    r=dict(evidence='uncalibrated_simulation',hardware_ready=False,development_seed=591191,configs=configs,sensor=sensor,history=[],
           source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
           [Path(__file__),Path('experiments/train_heading.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        for i,o in enumerate(pool.map(evaluate,[(gait,f,configs,8.,sensor) for f in candidates])):
            r['history'].append(o);r['best']=max(r['history'],key=lambda o:o['score']);path.write_text(json.dumps(r,allow_nan=False)+'\n')
            print(i,'score',round(o['score'],2),'pass',sum(m['feasible'] for m in o['metrics']),
                  'speed',[round(m['speed_mm_s'],1) for m in o['metrics']],flush=True)
    r['history'].sort(key=lambda o:o['score'],reverse=True);r['complete']=True
    path.write_text(json.dumps(r,allow_nan=False)+'\n');print('BEST',json.dumps(brief(r['best'])),flush=True)
if __name__=='__main__':main()
