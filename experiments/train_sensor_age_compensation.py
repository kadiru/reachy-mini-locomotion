"""Compensate known roll-observation age; no hidden motor/friction model input."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_penguin import selected_policy,make_job
from train_faster_balance import cases
from qualify_policy_bank import evaluate

if __name__=='__main__':
    base=selected_policy();base.update(target_step_turn_deg=12,max_repeats=4)
    base['sequencing']=dict(min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0.)
    jobs=[]
    for factor in [.5,1.,1.5]:
        p=copy.deepcopy(base)
        for family in ['primary','fallback']:p[family]['feedback']['sensor_latency_compensation']=factor
        for original in cases():
            if original['id']!='fresh-5':continue
            j=make_job(original,p,seconds=24);j['variant']=str(factor);jobs.append(j)
    out=Path('experiments/reports/motor-training-faster-balance-sensor-age-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    with ProcessPoolExecutor(max_workers=2) as pool:
        for j,c in zip(jobs,pool.map(evaluate,jobs)):
            c['variant']=j['variant'];report['checks'].append(c);out.write_text(json.dumps(report)+'\n')
            print(json.dumps(dict(factor=c['variant'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],speed=c['penguin']['speed_mm_s'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report)+'\n')
