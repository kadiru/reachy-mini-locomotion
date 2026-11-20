"""Use the new fallback only after the regular fallback actually turns too little."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_penguin import make_job
from train_faster_balance import cases,read
from qualify_policy_bank import evaluate

if __name__=='__main__':
    p=read('configs/penguin-faster-trained.json');p['rescue_after_weak_fallback_deg']=5.
    output_policy=Path('configs/penguin-faster-feedback-trained.json')
    if output_policy.exists():raise FileExistsError(output_policy)
    output_policy.write_text(json.dumps(p,indent=2)+'\n')
    jobs=[make_job(j,p,seconds=24) for j in cases() if j['id'] in ['fresh-5','fresh-7']]
    out=Path('experiments/reports/motor-training-faster-balance-progress-rescue-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,frozen_policy=p,jobs=jobs,checks=[])
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);out.write_text(json.dumps(report)+'\n');print(json.dumps(dict(case=c['id'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],speed=c['penguin']['speed_mm_s'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report)+'\n')
