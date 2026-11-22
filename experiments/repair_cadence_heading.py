"""Retrain on the observed course failure, then test fresh conditions on both starts."""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_cadence import read,rollout,save


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=4)
    ap.add_argument('--fit-only',action='store_true',help='Stop after selecting the walking policy, before qualification')
    a=ap.parse_args()
    if a.out.exists():
        ap.error('Choose a new report')
    old=read('experiments/reports/motor-training-alternating-edge-walks-2026-11-21.json')
    base=read('configs/cadence-alternating-experimental.json')
    model=read('configs/motors-speed-trained.json')
    hard=next(r for r in old['rows'] if not r['passed'])
    failed_model=old['conditions'][int(hard['name'].split('-')[-1])]
    policies={}
    for cap in (.12,.16,.20,.24,.28):
        for delay in (-.002,0.,.002):
            n=f'cap{cap}-delay{delay}'
            policies[n]=dict(base,pair_steering=dict(base['pair_steering'],delay_s_per_deg=delay,max_yaw_bias=cap))
    def rank(rows):
        return min((r['passed'], min(map(len,r['edge_brake_by_roll_sign'].values())) >= .35*r['planned_swings'],
                    r['mm_per_swing']) for r in rows)
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        fit=list(pool.map(rollout,[(n,p,hard['side'],16,failed_model,None) for n,p in policies.items()]))
        top=[r['name'] for r in sorted(fit,key=lambda r:rank([r]),reverse=True)[:3]]
        checks=list(pool.map(rollout,[(n,policies[n],s,24,dict(model,timestep_s=.00025),None)
                                      for n in top for s in (-1,1)]))
        name=max(top,key=lambda n:rank([r for r in fit+checks if r['name']==n]))
        p=policies[name]
        report=dict(evidence='MuJoCo only',fit=fit,nominal=checks,policy=dict(p,selected_candidate=name),
                    failed_condition=hard,parameters=policies)
        save(a.out,report)
        if a.fit_only:
            return
        def grid(conditions):
            return list(pool.map(rollout,[(f'floor-{i}',p,s,16,c,None) for i,c in enumerate(conditions) for s in (-1,1)]))
        report['retest']=grid(old['conditions'])
        rng=np.random.default_rng(2026112203)
        conditions=[]
        for i in range(5):
            c=copy.deepcopy(model);c['timestep_s']=.0005
            c['friction_slide_torsion_roll'][0]=float(rng.uniform(.55,.9))
            c['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),
                                    delay_frames=int(rng.integers(0,3)),mass_distribution=['uniform','torso'][i%2])
            conditions.append(c)
        report['fresh_conditions']=conditions
        report['fresh']=grid(conditions)
        report['long']=list(pool.map(rollout,[(name,p,s,40,dict(model,timestep_s=.00025),None) for s in (-1,1)]))
        save(a.out,report)


if __name__=='__main__':
    main()
