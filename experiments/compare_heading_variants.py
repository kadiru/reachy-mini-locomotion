"""Compare sine strokes and fast/conservative blends with heading feedback."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
from reachy_locomotion.sway import Gait
from reachy_locomotion.heading import HeadingFeedback,HeadingSensor
from select_progress import stress
from train_heading import evaluate,brief


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    base=json.loads(Path('configs/motors-datasheet.json').read_text());configs=[base]+stress(base)
    fast=asdict(Gait(**json.loads(Path('configs/sway-progress-fast.json').read_text())))
    conservative=asdict(Gait(**json.loads(Path('configs/sway-progress-conservative.json').read_text())))
    gaits=[dict(fast,yaw_sharpness=0,roll_sharpness=0)]
    for blend in [.25,.5,.75,1.]:gaits.append({k:(1-blend)*fast[k]+blend*conservative[k] for k in fast})
    old=json.loads(Path('experiments/reports/motor-training-heading-cem-2026-11-19.json').read_text())['best']['feedback']
    feedbacks=[dict(old,startup_ramp_cycles=2.),asdict(HeadingFeedback(kp=.2,ki=.03,kd=0,filter_tau_s=.12,startup_ramp_cycles=2)),
               asdict(HeadingFeedback(kp=.5,ki=.08,kd=0,filter_tau_s=.12,startup_ramp_cycles=2))]
    sensor=asdict(HeadingSensor());r=dict(evidence='uncalibrated_simulation',hardware_ready=False,configs=configs,sensor=sensor,history=[])
    with ProcessPoolExecutor(max_workers=4) as pool:
        for i,o in enumerate(pool.map(evaluate,[(g,f,configs,8.,sensor) for g in gaits for f in feedbacks])):
            r['history'].append(o);r['best']=max(r['history'],key=lambda o:o['score']);path.write_text(json.dumps(r)+'\n')
            print(i,'score',round(o['score'],2),'pass',sum(m['feasible'] for m in o['metrics']),
                  'speed',[round(m['speed_mm_s'],1) for m in o['metrics']],flush=True)
    r['history'].sort(key=lambda o:o['score'],reverse=True);r['complete']=True
    path.write_text(json.dumps(r)+'\n');print('BEST',json.dumps(brief(r['best'])),flush=True)
if __name__=='__main__':main()
