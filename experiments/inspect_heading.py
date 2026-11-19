"""Record a feedback rollout and distinguish drift from within-cycle yaw."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from reachy_locomotion.sway import Gait,SwaySimulator
from select_progress import stress

def main():
    p=argparse.ArgumentParser();p.add_argument('search');p.add_argument('--domain',type=int,default=1);p.add_argument('--output',required=True);a=p.parse_args()
    if Path(a.output).exists():p.error('Choose a new output path')
    src=json.loads(Path(a.search).read_text());best=src['best']
    base=json.loads(Path('configs/motors-datasheet.json').read_text());c=([base]+stress(base))[a.domain]
    r=SwaySimulator('experiments/upstream-reachy-mini',c).run(Gait(**best['gait']),cycles=16,warmup_cycles=3,record=True,feedback=best['feedback'])
    Path(a.output).write_text(json.dumps(r)+'\n')
    n=round(50/r['actual_frequency_hz']);t=r['trajectory']
    for i in range(len(t)//n):
        seg=t[i*n:(i+1)*n];yaw=np.array([x['heading_from_startup_rad']*180/math.pi for x in seg]);u=np.array([x['steering_correction_deg'] for x in seg])
        print(i,'yaw min mean max',np.round([yaw.min(),yaw.mean(),yaw.max()],2),'trim',np.round([u.min(),u.mean(),u.max()],2))
if __name__=='__main__':main()
