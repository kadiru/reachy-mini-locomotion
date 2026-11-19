"""Select fastest nominal gait with two-timestep long-run checks; stress test separately.

Selection never reads the search receipt's heldout evaluation. All candidate
ranking uses nominal development runs; stress cases are generated after freeze.
"""
import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_progress import evaluate,metrics
from reachy_locomotion.sway import Gait,SwaySimulator


def nominal_score(o):
    okay=all(m['feasible'] for m in o['metrics'])
    return (okay,min(m['speed_mm_s'] for m in o['metrics']))


def stress(base):
    rng=np.random.default_rng(591191)
    cases=[]
    for i in range(6):
        c=copy.deepcopy(base)
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
        c['foot_radius_m']=float(rng.uniform(.054,.058))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6.)),
            pid_scale=float(rng.uniform(.85,1.15)),delay_frames=int(rng.integers(0,3)),
            mass_distribution=['torso','uniform'][i%2],
            passive_friction_nm=float(rng.uniform(.0007,.002)),
            joint_friction_nm=float(rng.uniform(.007,.015)),
            joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
        cases.append(c)
    return cases


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('search');p.add_argument('--output',required=True);a=p.parse_args()
    path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    source=json.loads(Path(a.search).read_text())
    base=json.loads(Path('configs/motors-datasheet.json').read_text());fine=copy.deepcopy(base);fine['timestep_s']=.001
    unique={}
    # Nominal explore archive only; robust branch and all holdouts are excluded.
    for h in source['history']:
        if h['stage']=='explore':
            for c in h['candidates']:
                if c['feasible_all']:unique[json.dumps(c['gait'],sort_keys=True)]=c
    candidates=sorted(unique.values(),key=lambda c:c['score'],reverse=True)[:24]
    receipt={'evidence':'uncalibrated_simulation','hardware_ready':False,
        'selection':'fastest minimum nominal speed at 1 and 2ms with <=10deg max heading, drift <=30% forward+3mm, tilt<=18deg',
        'scope':'nominal_model_candidate; robustness reported separately; not universal success',
        'search_receipt':a.search,'stress_seed':591191,'stress_configs':stress(base)}
    with ProcessPoolExecutor(max_workers=4) as pool:
        validation=list(pool.map(evaluate,[(c['gait'],[base,fine],12.) for c in candidates]))
        validation.sort(key=nominal_score,reverse=True);receipt['validation_12s']=validation
        eligible=[c for c in validation if nominal_score(c)[0]][:6]
        if not eligible:raise RuntimeError('No nominal candidate meets 12-second criteria')
        long=list(pool.map(evaluate,[(c['gait'],[base,fine],20.) for c in eligible]))
        receipt['validation_20s']=long
        # Check extrapolated duration with the same heading constraint, not a
        # per-second allowance that would hide cumulative turning.
        long.sort(key=nominal_score,reverse=True)
        accepted=[c for c in long if nominal_score(c)[0]]
        chosen=accepted[0] if accepted else validation[0]
        receipt['passed_20s']=bool(accepted)
        receipt['frozen_policy']=chosen['gait'];receipt['selected_development']=chosen
        path.with_name(path.stem+'-policy.json').write_text(json.dumps(chosen['gait'],indent=2)+'\n')
        path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
        checks=[]
        for label,g in [('selected',chosen['gait']),('previous_smooth',json.loads(Path('configs/sway-trained.json').read_text())),
                        ('previous_fast',json.loads(Path('experiments/reports/motor-training-2026-11-18-policy.json').read_text()))]:
            for i,c in enumerate([base]+stress(base)):
                for dt in [.002,.001]:
                    c=copy.deepcopy(c);c['timestep_s']=dt;checks.append((label,i,g,c))
        outcomes=list(pool.map(evaluate,[(g,[c],12.) for _,_,g,c in checks]))
        receipt['evaluation']=[{'label':l,'domain_index':i,'result':o['results'][0],'metrics':o['metrics'][0]}
                               for (l,i,g,c),o in zip(checks,outcomes)]
        # Mechanism ablations use the frozen nominal policy; never select a new one.
        ablations={'no_yaw_stroke':dict(chosen['gait'],yaw_deg=0),
                   'no_lateral_stroke':dict(chosen['gait'],roll_deg=0,y_mm=0),
                   'sine_without_dwell':dict(chosen['gait'],yaw_sharpness=0,roll_sharpness=0),
                   'yaw_phase_plus_90':dict(chosen['gait'],yaw_phase_deg=chosen['gait']['yaw_phase_deg']+90)}
        outcomes=list(pool.map(evaluate,[(g,[base,fine],12.) for g in ablations.values()]))
        receipt['ablations']=[{'label':l,'trial':o} for l,o in zip(ablations,outcomes)]
    receipt['completed']=True;path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    # Save a representative nominal 1ms rollout, not a lucky stress configuration.
    recorded=SwaySimulator('experiments/upstream-reachy-mini',fine).run(Gait(**chosen['gait']),
        cycles=math.ceil(12/(round(50/chosen['gait']['frequency_hz'])/50)),warmup_cycles=3,record=True)
    recording=Path('experiments/generated')/(path.stem+'-states.json')
    if recording.exists():raise FileExistsError(recording)
    recording.write_text(json.dumps(recorded,allow_nan=False)+'\n')
    print('Completed '+str(path)+'; states '+str(recording),flush=True)


if __name__=='__main__':main()
