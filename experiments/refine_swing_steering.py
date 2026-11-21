"""Keep the learned stroke, fit heading correction on longer walks, then freeze."""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_swing_distance import read, save, trial


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    base=read(args.policy)
    candidates={'original':base}
    rng=np.random.default_rng(2026112105)
    for i in range(27):
        p=copy.deepcopy(base)
        p['feedback'].update(kp=float(rng.uniform(.035,.35)),ki=float(rng.uniform(0,.05)),
            max_correction_deg=float(rng.uniform(1.5,4.5)),heading_deadband_deg=float(rng.uniform(0,6)),
            roll_per_correction=float(rng.uniform(-1,1)),yaw_per_roll=float(rng.uniform(-2.5,1.5)))
        candidates[f'steering-{i:02}']=p
    nominal=read('configs/motors-speed-trained.json')
    sensor=read('configs/heading-sensor-assumed.json')
    history={}
    def batch(pool,jobs,label):
        rows=list(pool.map(trial,jobs))
        history[label]=rows
        save(args.output/'history.json',history)
        print(label,len(rows),sum(r['passed'] for r in rows),flush=True)
        return rows
    def rank(rows,name):
        matching=[r for r in rows if r['name']==name]
        return (sum(r['passed'] for r in matching),min(r['mm_per_swing'] for r in matching))
    def jobs(names,pairs,dt):
        return [(n,candidates[n],dict(nominal,timestep_s=dt),s,pairs,sensor,None) for n in names for s in (-1,1)]
    save(args.output/'candidates.json',candidates)
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=batch(pool,jobs(candidates,56,.0005),'development')
        finalists=sorted(candidates,key=lambda n:rank(rows,n),reverse=True)[:5]
        checks=batch(pool,jobs(finalists,24,.00025)+jobs(finalists,80,.00025),'confirm')
        chosen=max(finalists,key=lambda n:rank(checks,n))
        policy=dict(candidates[chosen],selected_candidate=chosen,stroke_candidate=base['selected_candidate'],
                    steering_seed=2026112105)
        save(args.output/'policy.json',policy)
        print('Frozen:',chosen,rank(checks,chosen),flush=True)
        fresh=np.random.default_rng(2026112106)
        trials=[]
        conditions=[]
        for i in range(6):
            config=copy.deepcopy(nominal)
            config['friction_slide_torsion_roll'][0]=float(fresh.uniform(.55,.9))
            config['motor_model'].update(voltage_v=float(fresh.uniform(4.7,5.5)),pid_scale=float(fresh.uniform(.9,1.1)),
                delay_frames=int(fresh.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
            sensing=dict(sensor,seed=int(fresh.integers(100000)),delay_frames=int(fresh.integers(1,4)))
            config['timestep_s']=.0005
            conditions.append(dict(config=config,sensor=sensing))
            for side in (-1,1):
                trials.append((chosen,policy,config,side,24,sensing,None))
        save(args.output/'stress-conditions.json',conditions)
        batch(pool,trials,'heldout')
        batch(pool,jobs([chosen],80,.000125),'fine-long')


if __name__=='__main__':
    main()
