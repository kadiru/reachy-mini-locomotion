"""Start yaw before the edge catch so bounded motors can brake on the edge."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--margin',type=float,default=0);args=p.parse_args()
    out=Path('experiments/reports/motor-training-balance-anticipatory-margin-2026-11-19.json' if args.margin else 'experiments/reports/motor-training-balance-anticipatory-pivot-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    seed=next(c for c in read('experiments/reports/motor-training-balance-lateral-stress-2026-11-19.json')['candidates'] if c['id']=='lateral-4');jobs=[]
    for i,(pred,trigger,yaw) in enumerate(itertools.product([.04,.1,.16],[2,6],[-40,40,60])):
        f=dict(seed['feedback'],yaw_before_catch=True,yaw_direction=-1,yaw_prediction_s=pred,workspace_joint_margin_rad=args.margin)
        g=dict(seed['recipe'],yaw_at_roll_deg=trigger,yaw_s=.08,yaw_deg=yaw)
        for side,model in itertools.product([-1,1],[3,4]):jobs.append((f'anticipatory-{i}',g,f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
