"""Tune the signed roll/rate catch boundary for sufficient edge-supported turn."""
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    out=Path('experiments/reports/motor-training-balance-pivot-energy-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    seed=next(c for c in read('experiments/reports/motor-training-balance-lateral-stress-2026-11-19.json')['candidates'] if c['id']=='lateral-4');jobs=[]
    for i,(pred,trigger) in enumerate(itertools.product([.02,.06,.1],[10,12,14,16,18])):
        f=dict(seed['feedback'],prediction_s=pred,trigger_deg=trigger)
        for side,model in itertools.product([-1,1],[3,4]):jobs.append((f'energy-{i}',seed['recipe'],f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
