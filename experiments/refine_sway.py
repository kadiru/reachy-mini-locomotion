"""Refine the first trained gait for worst-domain progress, heading and smoothness."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_sway import KEYS, LOW, HIGH, domains, evaluate


def robust_fitness(r):
    if not r['valid']:return -100.
    t=r['measured_seconds']
    return (1000*r['forward_m_per_s']-1500*abs(r['lateral_m'])/t
            -50*abs(r['heading_change_rad'])/t
            -r.get('rms_base_acceleration_m_s2',0)
            -r.get('rms_base_angular_velocity_rad_s',0)
            -.5*max(r['rms_motor_torque_nm'][1:7])
            -5*r['any_motor_torque_saturated_fraction'])


def trial(payload):
    out=evaluate(payload)
    scores=np.array([robust_fitness(r) for r in out['results']])
    out['domain_scores']=scores.tolist()
    out['score']=float(.8*np.min(scores)+.2*np.mean(scores))
    return out


def new_holdout(base):
    # New unseen domains; earlier study's evaluation became development evidence.
    rng=np.random.default_rng(67190)
    result=[]
    for i in range(6):
        c=copy.deepcopy(base)
        c['foot_radius_m']=float(rng.uniform(.054,.059))
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.45,1.))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.3,6.)),
            torque_derating=float(rng.uniform(.7,.85)),pid_scale=float(rng.uniform(.8,1.3)),
            delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
            passive_friction_nm=float(rng.uniform(.0005,.003)),
            joint_friction_nm=float(rng.uniform(.005,.02)),
            joint_damping_nm_s_rad=float(rng.uniform(.0025,.01)),
            joint_armature_kg_m2=float(rng.uniform(.00005,.0002)))
        result.append(c)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--generations',type=int,default=12)
    p.add_argument('--population',type=int,default=24)
    p.add_argument('--seed',type=int,default=20261119)
    args=p.parse_args();path=Path(args.output)
    if path.exists():p.error('Choose a new receipt')
    if not 1<=args.generations<=30 or not 8<=args.population<=64:p.error('Bounded experiment')
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    train=domains(base);nominal_fine=copy.deepcopy(base);nominal_fine['timestep_s']=.001
    train.append(nominal_fine);train[2]['timestep_s']=.001
    previous=json.loads(Path('experiments/reports/motor-training-2026-11-18-policy.json').read_text())
    rng=np.random.default_rng(args.seed)
    mean=np.array([previous[k] for k in KEYS]);std=.18*(HIGH-LOW)
    best=None;history=[]
    receipt={'evidence':'uncalibrated_simulation','hardware_ready':False,
        'method':'CEM_worst_domain_heading_smoothness_refinement','seed':args.seed,
        'holdout_seed':67190,'training_cycles':10,'evaluation_cycles':24,
        'generations':args.generations,'population':args.population,
        'training_configs':train,'evaluation_configs':new_holdout(base),'history':history,
        'code_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
            [Path(__file__),Path('experiments/train_sway.py'),Path('src/reachy_locomotion/motors.py'),Path('src/reachy_locomotion/sway.py')]}}
    with ProcessPoolExecutor(max_workers=4) as pool:
        for gen in range(args.generations):
            samples=np.clip(rng.normal(mean,std,size=(args.population,len(KEYS))),LOW,HIGH)
            samples[0]=[best['gait'][k] for k in KEYS] if best else mean
            samples[-3:]=rng.uniform(LOW,HIGH,size=(3,len(KEYS)))
            out=list(pool.map(trial,[(dict(zip(KEYS,row.tolist())),train,10) for row in samples]))
            order=np.argsort([r['score'] for r in out])[::-1]
            elites=samples[order[:max(4,args.population//4)]]
            mean=.2*mean+.8*elites.mean(axis=0)
            std=np.maximum(.2*std+.8*elites.std(axis=0),.025*(HIGH-LOW))
            if best is None or out[order[0]]['score']>best['score']:best=out[order[0]]
            history.append({'generation':gen,'candidates':out})
            receipt['best_training']=best
            path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
            print(json.dumps({'generation':gen,'score':best['score'],
              'forward_mm_s':[round(r['forward_m_per_s']*1000,3) for r in best['results']],
              'yaw_deg':[round(np.degrees(r['heading_change_rad']),2) for r in best['results']]}),flush=True)
        controls={'refined':best['gait'],'initial_trained':previous,
                  'old_untrained':json.loads(Path('configs/sway-candidate.json').read_text()),
                  'stationary':{'frequency_hz':2.,'roll_deg':0.,'yaw_deg':0.}}
        checks=[]
        for label,g in controls.items():
            for idx,c in enumerate([base]+receipt['evaluation_configs']):
                for dt in [.002,.001]:
                    c=copy.deepcopy(c);c['timestep_s']=dt
                    checks.append((label,idx,g,c))
        out=list(pool.map(trial,[(g,[c],24) for _,_,g,c in checks]))
        receipt['evaluation']=[dict(label=l,domain_index=i,result=o['results'][0])
                              for (l,i,g,c),o in zip(checks,out)]
    receipt['completed']=True
    path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    path.with_name(path.stem+'-policy.json').write_text(json.dumps(best['gait'],indent=2)+'\n')
    print('Completed '+str(path),flush=True)


if __name__=='__main__':main()
