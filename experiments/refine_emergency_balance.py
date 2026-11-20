"""Promote a newly discovered guard failure to development; retain the frozen result."""
import copy
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from qualify_policy_bank import evaluate


def main():
    source=json.loads(Path('experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json').read_text())
    # First-burst recovery screen on the two newly failed models and nominal.
    # A full pivot/walking qualification must follow any promising result.
    base=[j for j in source['frozen_jobs'] if j['pivot_only'] and j['id'] in ('fresh-3','fresh-7','development-0')]
    jobs=[]
    for threshold in [15,20,25]:
        for prediction in [.04,.08]:
            for duration in [.08,.16]:
                for original in base:
                    j=copy.deepcopy(original);j['seconds']=6
                    j['config']['timestep_s']=.0005
                    j['policy']['target_step_turn_deg']=0;j['policy']['max_repeats']=1
                    j['variant']=f'{threshold}-{prediction}-{duration}'
                    for name in ['primary','fallback']:
                        j['policy'][name]['feedback'].update(emergency_roll_deg=threshold,emergency_prediction_s=prediction,emergency_blend_s=duration)
                    jobs.append(j)
    path=Path('experiments/reports/motor-training-balance-emergency-screen-2026-11-19.json')
    if path.exists():raise FileExistsError(path)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    with ProcessPoolExecutor(max_workers=4) as pool:
        for j,c in zip(jobs,pool.map(evaluate,jobs)):
            c['variant']=j['variant'];report['checks'].append(c);path.write_text(json.dumps(report)+'\n')
            print(j['variant'],j['id'],j['side'],c['result']['recovered'],c['result']['termination'],round(c['result']['final_foot_yaw_deg'],1),flush=True)
    report['completed']=True;path.write_text(json.dumps(report)+'\n')

if __name__=='__main__':main()
