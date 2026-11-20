"""Check measured head completion before restarting a shortened primitive."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_penguin import selected_policy,make_job
from train_faster_balance import cases
from qualify_policy_bank import evaluate

if __name__=='__main__':
    p=selected_policy();p.update(target_step_turn_deg=10,max_repeats=4)
    p['sequencing']=dict(min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0.,head_settle_yaw_deg=2,head_settle_rate_deg_s=15)
    jobs=[make_job(j,p,seconds=24) for j in cases() if j['id'] in ['development-0','fresh-5','fresh-7']]
    out=Path('experiments/reports/motor-training-faster-balance-head-settling-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    with ProcessPoolExecutor(max_workers=2) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);out.write_text(json.dumps(report)+'\n');print(json.dumps(dict(case=c['id'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],speed=c['penguin']['speed_mm_s'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report)+'\n')
