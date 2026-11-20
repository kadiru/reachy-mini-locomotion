"""Separate head-roll momentum shaping from lateral COM balance correction."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--negative',action='store_true');args=parser.parse_args()
    out=Path('experiments/reports/motor-training-balance-lateral-negative-2026-11-19.json' if args.negative else 'experiments/reports/motor-training-balance-lateral-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    seed=next(c for c in read('experiments/reports/motor-training-balance-signed-2026-11-19.json')['candidates'] if c['id']=='pivot-5-signed-4')
    jobs=[]
    grid=itertools.product([-1.5],[-.5,-.9,-1.3],[0,.04,.08]) if args.negative else itertools.product([-1.5,0],[.5,1.,1.5],[0,.06])
    for i,(kp,ykp,ykd) in enumerate(grid):
        f=dict(seed['feedback'],roll_kp=kp,y_per_roll_mm_deg=0,y_roll_kp_mm_deg=ykp,y_roll_kd_mm_s_deg=ykd)
        for side,model in itertools.product([-1,1],[0,1,2]):jobs.append((f'lateral-{i}',seed['recipe'],f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
