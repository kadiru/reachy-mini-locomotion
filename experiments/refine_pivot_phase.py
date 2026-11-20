"""Tune anticipatory yaw on the two remaining weak-pivot development models."""
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    out=Path('experiments/reports/motor-training-balance-pivot-phase-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    source=read('experiments/reports/motor-training-balance-lateral-stress-2026-11-19.json')
    seed=next(c for c in source['candidates'] if c['id']=='lateral-4');jobs=[]
    for i,(pred,stroke,yaw) in enumerate(itertools.product([.04,.12,.2],[.08,.16],[40,60])):
        f=dict(seed['feedback'],yaw_prediction_s=pred)
        g=dict(seed['recipe'],yaw_s=stroke,yaw_deg=yaw)
        for side,model in itertools.product([-1,1],[3,4]):jobs.append((f'phase-{i}',g,f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
