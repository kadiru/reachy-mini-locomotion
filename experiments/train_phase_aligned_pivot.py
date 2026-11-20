"""Choose yaw direction from the sensed rocking edge, with neutral recovery."""
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

def main():
    out=Path('experiments/reports/motor-training-balance-phase-aligned-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    seed=read('configs/balance-recovery-trained.json');jobs=[]
    for i,(trigger,yaw) in enumerate(itertools.product([4,6,8],[30,45,60])):
        f=dict(seed['feedback'],yaw_before_catch=True,yaw_direction=0,yaw_prediction_s=.06,yaw_align_with_roll=True,
               state_dependent_yaw=True,workspace_joint_margin_rad=.05)
        g=dict(seed['recipe'],yaw_at_roll_deg=trigger,yaw_s=.08,yaw_deg=yaw,yaw_hold_s=.2,yaw_return_s=1.)
        for side,model in itertools.product([-1,1],[0,3,4]):jobs.append((f'aligned-{i}',g,f,side,model,.0005,6.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,candidates=[],
                turn_direction_rule='Opposite the yaw target chosen from the delayed sensed rocking edge; assess mirror pairs separately')
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();r=c['result']
            print(json.dumps(dict(id=c['id'],side=c['side'],model=c['model_id'],recovered=r['recovered'],pivot_pass=signed_pivot(c),
                                 peak=r['peak_roll_deg'],edge_yaw=r['edge_supported_foot_yaw_deg'],final_yaw=r['final_foot_yaw_deg'],termination=r['termination'])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
