"""Recheck a recovery override on original development and newly exposed failures."""
import copy
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from qualify_policy_bank import evaluate


def main():
    original=json.loads(Path('experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json').read_text())
    assert original['completed']
    selected=dict(emergency_roll_deg=25,emergency_prediction_s=.04,emergency_blend_s=.16)
    jobs=[]
    for j in original['frozen_jobs']:
        if not (j['id'].startswith('development-') or j['id'] in ('fresh-3','fresh-5','fresh-7')):continue
        j=copy.deepcopy(j);j['config']['timestep_s']=.0005
        for name in ['primary','fallback']:j['policy'][name]['feedback'].update(selected)
        jobs.append(j)
    out=Path('experiments/reports/motor-training-policy-bank-emergency-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,
                selected_override=selected,original_fresh_failures_promoted_to_development=True,frozen_jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save()
            print(json.dumps(dict(id=c['id'],side=c['side'],pivot=c['pivot_only'],passed=c['passed'],recovered=c['result']['recovered'],
                 complete=c['penguin']['completed'],termination=c['result']['termination'],bursts=len(c['penguin']['bursts']),
                 yaw=c['result']['final_foot_yaw_deg'],edge=c['result']['edge_supported_foot_yaw_deg'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
