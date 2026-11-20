"""Remove the energy-injecting catch hold and tune the closed-loop neutral return."""
import copy
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from qualify_balance import evaluate
from train_balance import read,models,recovery_score

def main():
    out=Path('experiments/reports/motor-training-balance-neutral-recovery-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    first=read('experiments/reports/motor-training-balance-recovery-qualified-2026-11-19.json')
    second=read('experiments/reports/motor-training-balance-recovery-final-2026-11-19.json')
    if not second['completed']:raise ValueError('Wait for qualification')
    seed=first['frozen_candidate'];g=dict(seed['recipe'],catch_roll_deg=0,catch_y_mm=0,catch_s=.12,catch_hold_s=0)
    scenarios=[dict(config=models()[i],feedback=seed['feedback']) for i in [0,2]]
    for report in [first,second]:
        failed=next(c for c in report['checks'] if not c['result']['recovered'])
        scenarios.append(next(j for j in report['frozen_jobs'] if j['id']==failed['id'] and j['side']==failed['side']))
    jobs=[]
    for i,(pred,kp,kd) in enumerate(itertools.product([.1,.16],[0,1.5],[-.12,0,.12])):
        f=dict(seed['feedback'],prediction_s=pred,roll_kp=kp,roll_kd_s=kd,workspace_projection=True)
        for model,scenario in enumerate(scenarios):
            config=copy.deepcopy(scenario['config']);config['timestep_s']=.0005
            ff=dict(f)
            for key in ['sensor_delay_frames','sensor_noise_deg','sensor_seed']:ff[key]=scenario['feedback'][key]
            for side in [-1,1]:jobs.append((f'neutral-{i}-model-{model}',g,ff,side,config))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                validation_failures_promoted_to_training=True,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            c['condition_id']=c['id'];c['id']=c['id'].split('-model-')[0]
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['condition_id'],side=c['side'],score=recovery_score(r),roll=r['peak_roll_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
