"""Local motor/contact dynamics identification from disclosed artificial starts."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.edge_linear import LinearEdgeController
from reachy_locomotion.edge_balance import EdgeFeedback
from reachy_locomotion.primitives import PrimitiveSimulator
from train_edge_balance import read

CACHE={}
def sample(job):
    seed,dt=job
    if dt not in CACHE:
        config=read('configs/motors-speed-trained.json');config['timestep_s']=dt
        CACHE[dt]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    s=CACHE[dt];policy=dict(recipe=read('configs/primitive-catch-experimental.json'),feedback=asdict(EdgeFeedback(target_roll_deg=32.1,sensor_delay_frames=0)))
    c=LinearEdgeController(policy['recipe'],policy['feedback'],-1,s.checked_ik,excitation_seed=seed);c.hold_from_start=True
    rng=np.random.default_rng(seed)
    r=s.run_primitive(policy['recipe'],seconds=2,controller=c,waist_mode='hold_neutral',tilt_limit_deg=None,diagnostic_initial_roll_deg=-32.1+float(rng.uniform(-.8,.8)))
    return dict(seed=seed,termination=r['termination'],trace=[{k:x[k] for k in ['time_s','edge_state','edge_action','edge_observation']} for x in r['trace']])

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--count',type=int,default=64);p.add_argument('--timestep',type=float,default=.0005);a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    rows=[]
    with ProcessPoolExecutor(max_workers=6) as pool:
        for r in pool.map(sample,[(seed,a.timestep) for seed in range(2026112100,2026112100+a.count)]):rows.append(r)
    out.write_text(json.dumps(dict(evidence='uncalibrated_simulation',artificial_starts=True,timestep_s=a.timestep,rows=rows))+'\n')
    print('samples',sum(len(r['trace']) for r in rows),flush=True)

if __name__=='__main__':main()
