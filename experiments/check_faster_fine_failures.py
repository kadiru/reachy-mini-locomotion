"""Repeat the formerly failing walking cases at 0.125 ms, without retuning."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_penguin import make_job
from train_faster_balance import cases,read
from qualify_policy_bank import evaluate

if __name__=='__main__':
    p=read('configs/penguin-faster-feedback-trained.json')
    jobs=[make_job(j,p,seconds=24,dt=.000125) for j in cases() if (j['id'],j['side']) in [('fresh-3',-1),('fresh-5',1),('fresh-7',-1),('fresh-7',1)]]
    out=Path('experiments/reports/motor-training-faster-balance-finer-failures-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=2) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(case=c['id'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'])),flush=True)
    report['completed']=True;save()
