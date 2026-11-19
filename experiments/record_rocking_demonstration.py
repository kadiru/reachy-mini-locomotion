"""Freeze the best qualified 20-second demo without claiming long-run success.

Both original and provisional long-run selections failed. This artifact keeps
those failures explicit and records the best twenty-second, two-timestep trial.
"""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait,SwaySimulator
from reachy_locomotion.rocking import rocking_metrics
from select_rocking import stress_job
from train_rocking import read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selection',required=True);p.add_argument('--output',required=True)
    p.add_argument('--recording',required=True);args=p.parse_args()
    out=Path(args.output);recording=Path(args.recording)
    if out.exists() or recording.exists():p.error('Preserve prior evidence')
    source=read(args.selection)
    accepted=[c for c in source['fine'] if c['accepted']]
    if not accepted:p.error('No qualifying twenty-second demonstration')
    best=max(accepted,key=lambda c:c['conservative_stride_mm'])
    recipe=dict(id=best['id'],gait=best['gait'],feedback=best['feedback'])
    report=copy.deepcopy(source)
    report.update(completed=False,args=vars(args),frozen_recipe=recipe,stress=[],stress_seed=2026111933,
                  acceptance_mode='20-second demonstration only; forty-second continuous qualification failed',
                  long_horizon_qualified=False,original_style_gates_pass_all_selection_checks=False,
                  source_selection_sha256=hashlib.sha256(Path(args.selection).read_bytes()).hexdigest(),
                  recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    g=Gait(**recipe['gait']);period=round(50/g.frequency_hz)/50
    sim=SwaySimulator('experiments/upstream-reachy-mini',read('configs/motors-speed-trained.json'))
    r=sim.run(g,cycles=math.ceil(20/period-1e-10),warmup_cycles=3,record=True,feedback=recipe['feedback'],
              sensor=read('configs/heading-sensor-assumed.json'))
    expected=next(c['result'] for c in source['development'] if c['id']==recipe['id'])
    for k in ['measurement_displacement_in_startup_frame_m','rocking_cycles','max_foot_tilt_deg','negative_motor_work_j']:
        assert r[k]==expected[k],k
    r.update(render_forward_frame='startup',evaluation_heading_rule='completed_cycle',
             qualification='20-second demonstration; long-run steering qualification failed')
    recording.write_text(json.dumps(r,allow_nan=False)+'\n')
    Path('configs/sway-rocking-experimental.json').write_text(json.dumps(recipe['gait'],indent=2)+'\n')
    Path('configs/heading-rocking-experimental.json').write_text(json.dumps(recipe['feedback'],indent=2)+'\n')
    report['nominal']={k:v for k,v in r.items() if k!='trajectory'};report['nominal_metrics']=rocking_metrics(r)
    report['recorded_replay_matches_selection_exactly']=True
    report['recording_sha256']=hashlib.sha256(recording.read_bytes()).hexdigest();save()
    print(json.dumps(dict(stage='recorded',**report['nominal_metrics'])),flush=True)
    rng=np.random.default_rng(report['stress_seed']);jobs=[]
    for i in range(8):
        c=read('configs/motors-speed-trained.json');c['foot_radius_m']=float(rng.uniform(.054,.058))
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6)),pid_scale=float(rng.uniform(.85,1.15)),
                               delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
                               passive_friction_nm=float(rng.uniform(.0007,.002)),joint_friction_nm=float(rng.uniform(.007,.015)),
                               joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
        for dt in [.001,.0005]:
            variant=copy.deepcopy(c);variant['timestep_s']=dt;jobs.append((f'fresh-{i}-{dt}',recipe,variant,20.))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(stress_job,jobs):
            report['stress'].append(c);save();print(json.dumps(dict(stage='fresh',id=c['id'],**c['metrics'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
