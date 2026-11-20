"""Test a physical neutral return between steps, never reset the robot state."""
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read
from train_penguin import evaluate

def main():
    out=Path('experiments/reports/motor-training-penguin-neutral-return-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    source=read('experiments/reports/motor-training-penguin-probe-2026-11-19.json')
    seed=max(source['candidates'],key=lambda c:c['penguin']['score']);jobs=[]
    for delay,model,side in itertools.product([1.,1.5,2.],range(5),[-1,1]):
        f=dict(seed['feedback'],return_x_after_catch_s=delay,return_x_duration_s=.6,workspace_projection=True)
        jobs.append((f"neutral-return-{delay}",seed['recipe'],f,model,.0005,2,3.,side))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            print(json.dumps(dict(id=c['id'],model=c['model_id'],side=c['start_side'],termination=c['result']['termination'],**c['penguin'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
