"""Record frozen nominal and predeclared stress-case demonstrations, no reselection."""
import argparse
import copy
import json
import math
from pathlib import Path
import numpy as np
from reachy_locomotion.sway import Gait,SwaySimulator
from select_progress import stress
from train_progress import metrics


def main():
    p=argparse.ArgumentParser();p.add_argument('evaluation');p.add_argument('--output',required=True);a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    evaluation=json.loads(Path(a.evaluation).read_text());policy=evaluation['frozen_policy']
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    old=json.loads(Path('configs/sway-progress-fast.json').read_text())
    receipt=dict(evidence='uncalibrated_simulation',hardware_ready=False,evaluation=a.evaluation,demonstrations=[])
    for label,c in [('nominal',base),('stress1',stress(base)[0])]:
        c=copy.deepcopy(c);c['timestep_s']=.001
        sim=SwaySimulator('experiments/upstream-reachy-mini',c);g=Gait(**policy['gait']);cycles=math.ceil(20/(round(50/g.frequency_hz)/50))
        r=sim.run(g,cycles=cycles,warmup_cycles=3,record=True,feedback=policy['feedback'],sensor=evaluation['sensor'])
        states=Path('experiments/generated/heading-feedback-'+label+'-2026-11-19.json')
        if states.exists():raise FileExistsError(states)
        states.write_text(json.dumps(r,allow_nan=False)+'\n')
        q=np.array([row['qpos'] for row in r['trajectory']])[:,sim.joint_qpos]
        margin=float(np.min(np.minimum(q-sim.command_limits[:,0],sim.command_limits[:,1]-q)))
        summary={k:v for k,v in r.items() if k!='trajectory'}
        summary['minimum_actual_joint_limit_margin_rad']=margin
        baseline=sim.run(Gait(**old),cycles=math.ceil(20/(round(50/old['frequency_hz'])/50)),warmup_cycles=3)
        receipt['demonstrations'].append(dict(label=label,states=str(states),result=summary,metrics=metrics(r),baseline=baseline))
        print(label,'speed',round(r['forward_m_per_s']*1000,3),'turn',round(np.degrees(r['max_heading_excursion_rad']),3),flush=True)
    path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    for filename,key in [('configs/sway-heading-trained.json','gait'),('configs/heading-trained.json','feedback')]:
        dest=Path(filename)
        if dest.exists():raise FileExistsError(dest)
        dest.write_text(json.dumps(policy[key],indent=2)+'\n')
    dest=Path('configs/heading-sensor-assumed.json')
    if dest.exists():raise FileExistsError(dest)
    dest.write_text(json.dumps(evaluation['sensor'],indent=2)+'\n')
if __name__=='__main__':main()
