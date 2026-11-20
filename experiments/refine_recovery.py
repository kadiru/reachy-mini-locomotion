"""Promote the failed validation condition to development, then tune prediction."""
import copy
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from qualify_balance import evaluate
from train_balance import read,models,recovery_score

def main():
    out=Path('experiments/reports/motor-training-balance-recovery-refined-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    old=read('experiments/reports/motor-training-balance-recovery-qualified-2026-11-19.json')
    seed=old['frozen_candidate'];g=seed['recipe'];jobs=[]
    stress=next(j for j in old['frozen_jobs'] if j['id']=='fresh-6' and j['side']==1)
    for i,(pred,trigger,kd) in enumerate(itertools.product([.08,.12,.16],[6,8],[0,.15])):
        f=dict(seed['feedback'],prediction_s=pred,trigger_deg=trigger,roll_kd_s=kd)
        for model_id,config in enumerate([models()[0],models()[2],stress['config']]):
            config=copy.deepcopy(config);config['timestep_s']=.0005
            ff=dict(f)
            if model_id==2:
                for key in ['sensor_delay_frames','sensor_noise_deg','sensor_seed']:ff[key]=stress['feedback'][key]
            for side in [-1,1]:jobs.append((f'prediction-{i}-model-{model_id}',g,ff,side,config))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                previous_fresh_condition_promoted_to_training='fresh-6; final fresh seed must change',candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            c['condition_id']=c['id'];c['id']=c['id'].split('-model-')[0]
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['condition_id'],side=c['side'],score=recovery_score(r),roll=r['peak_roll_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
