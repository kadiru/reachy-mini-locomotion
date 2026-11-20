"""Reduce unnecessary turning excursions while preserving the cycle heading gate."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_penguin import selected_policy,make_job
from train_faster_balance import cases
from qualify_policy_bank import evaluate


def main():
    base=selected_policy();jobs=[]
    base['sequencing']=dict(min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0.)
    for target in [10,12]:
        p=dict(base,target_step_turn_deg=target,max_repeats=4)
        for original in cases():
            if original['id'] not in ['development-0','fresh-5','fresh-7']:continue
            j=make_job(original,p,seconds=24);j['variant']=f'target-{target}';jobs.append(j)
    out=Path('experiments/reports/motor-training-faster-balance-heading-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for j,c in zip(jobs,pool.map(evaluate,jobs)):
            c['variant']=j['variant'];report['checks'].append(c);save();print(json.dumps(dict(variant=c['variant'],case=c['id'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],speed=c['penguin']['speed_mm_s'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'],bursts=len(c['penguin']['bursts']))),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
