"""Bounded CPU cross-entropy gait optimization; no hardware or RL claim.

Training domains are fixed before search. Evaluation domains never select policy.
Every candidate, rejection and score is retained in the JSON receipt.
"""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator

KEYS = ['frequency_hz','roll_deg','yaw_deg','yaw_phase_deg','pitch_bias_deg',
        'x_mm','y_mm','y_phase_deg','z_mm']
LOW = np.array([.7, 5, 0, -180, -12, -18, -20, -180, -10.])
HIGH = np.array([3.5, 40, 45, 180, 12, 18, 20, 180, 10.])
CACHE = {}


def domains(base, heldout=False):
    # Domain values are hypotheses, not measured tolerances or probabilities.
    settings = ([(5., .7, .8, 1., 1, 'torso', .056, .001),
                 (4.5, .5, .7, .8, 2, 'uniform', .058, .002),
                 (6., .9, .8, 1.3, 0, 'torso', .054, .0005)] if not heldout else
                [(v, mu, .75, gain, delay, mass, radius, pf)
                 for v, mu, gain, delay, mass, radius, pf in [
                     (4.7,.6,.9,1,'torso',.055,.0015),
                     (5.5,.8,1.1,2,'uniform',.057,.0008),
                     (5.8,.45,1.2,0,'torso',.058,.0025),
                     (4.2,.95,.75,2,'uniform',.054,.003),
                     (5.2,1.,1.4,1,'torso',.060,.001),
                     (5.,.4,.85,3,'uniform',.056,.002)]])
    result=[]
    for v,mu,derating,gain,delay,mass,radius,pf in settings:
        c=copy.deepcopy(base)
        c['friction_slide_torsion_roll'][0]=mu
        c['foot_radius_m']=radius
        c['motor_model'].update(voltage_v=v,torque_derating=derating,pid_scale=gain,
                                delay_frames=delay,mass_distribution=mass,passive_friction_nm=pf)
        result.append(c)
    return result


def fitness(r):
    if not r['valid']:
        return -100.
    t=r['measured_seconds']
    # mm/s reward, lateral/turning/impact/torque penalties. No rewards for rocking alone.
    rms=max(r['rms_motor_torque_nm'][1:7])
    return (1000*r['forward_m_per_s'] - 400*abs(r['lateral_m'])/t
            - 2*abs(r['heading_change_rad'])/t
            - .15*r.get('rms_base_acceleration_m_s2',0)
            - .05*rms - .05*r['any_motor_torque_saturated_fraction'])


def evaluate(payload):
    gait_dict, configs, cycles = payload
    results=[]
    for config in configs:
        key=json.dumps(config,sort_keys=True)
        if key not in CACHE:
            CACHE[key]=SwaySimulator('experiments/upstream-reachy-mini',config)
        results.append(CACHE[key].run(Gait(**gait_dict),cycles=cycles))
    scores=np.array([fitness(r) for r in results])
    return {'gait':gait_dict,'score':float(.5*np.mean(scores)+.5*np.min(scores)),
            'domain_scores':scores.tolist(),'results':results}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='configs/motors-datasheet.json')
    p.add_argument('--output',required=True)
    p.add_argument('--generations',type=int,default=10)
    p.add_argument('--population',type=int,default=24)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--seed',type=int,default=20261118)
    args=p.parse_args()
    path=Path(args.output)
    if path.exists():p.error('Choose a new report path')
    if not 1<=args.generations<=30 or not 8<=args.population<=64 or not 1<=args.workers<=4:
        p.error('Bounded local experiment: 1–30 generations, 8–64 population, 1–4 workers')
    base=json.loads(Path(args.config).read_text())
    train=domains(base)
    rng=np.random.default_rng(args.seed)
    mean=(LOW+HIGH)/2;mean[3]=90;mean[7]=0
    std=(HIGH-LOW)*.3
    history=[];best=None
    receipt={'schema_version':1,'evidence':'uncalibrated_simulation','hardware_ready':False,
             'method':'cross_entropy_gait_parameter_optimization_not_RL',
             'seed':args.seed,'bounds':dict(zip(KEYS,zip(LOW.tolist(),HIGH.tolist()))),
             'training_configs':train,'evaluation_configs':domains(base,True),
             'generations':args.generations,'population':args.population,'history':history,
             'code_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                            [Path(__file__),Path('src/reachy_locomotion/motors.py'),Path('src/reachy_locomotion/sway.py')]}}
    path.parent.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for generation in range(args.generations):
            samples=np.clip(rng.normal(mean,std,size=(args.population,len(KEYS))),LOW,HIGH)
            if best is not None:samples[0]=[best['gait'][k] for k in KEYS]
            # Maintain exploration instead of collapsing onto numerical creep.
            samples[-4:]=rng.uniform(LOW,HIGH,size=(4,len(KEYS)))
            payload=[(dict(zip(KEYS,row.tolist())),train,6) for row in samples]
            evaluated=list(pool.map(evaluate,payload))
            order=np.argsort([r['score'] for r in evaluated])[::-1]
            elites=samples[order[:max(4,args.population//4)]]
            mean=.25*mean+.75*elites.mean(axis=0)
            std=np.maximum(.25*std+.75*elites.std(axis=0),.04*(HIGH-LOW))
            winner=evaluated[order[0]]
            if best is None or winner['score']>best['score']:best=winner
            history.append({'generation':generation,'candidates':evaluated})
            receipt['best_training']=best
            path.write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
            print(json.dumps({'generation':generation,'score':best['score'],
                  'forward_mm_s':[round(r['forward_m_per_s']*1000,4) for r in best['results']],
                  'valid_candidates':sum(all(r['valid'] for r in c['results']) for c in evaluated)}),flush=True)
        # Freeze the policy BEFORE evaluation. These results do not feed search.
        controls={'trained':best['gait'], 'previous':json.loads(Path('configs/sway-candidate.json').read_text()),
                  'stationary':{'frequency_hz':2.,'roll_deg':0.,'yaw_deg':0.}}
        checks=[]
        for label,gait in controls.items():
            for domain_index,c in enumerate([base]+domains(base,True)):
                for dt in [.002,.001]:
                    c=copy.deepcopy(c);c['timestep_s']=dt
                    checks.append((label,domain_index,c,gait))
        outcomes=list(pool.map(evaluate,[(g,[c],16) for _,_,c,g in checks]))
        receipt['evaluation']=[dict(label=label,domain_index=di,result=o['results'][0])
                               for (label,di,c,g),o in zip(checks,outcomes)]
    receipt['completed']=True
    path.write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    policy=path.with_name(path.stem+'-policy.json')
    policy.write_text(json.dumps(best['gait'],indent=2)+'\n')
    print('Completed; frozen policy: '+str(policy),flush=True)


if __name__=='__main__':main()
