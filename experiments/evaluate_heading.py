"""Select on known stress domains, freeze, then evaluate fresh models/sensors."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
import numpy as np
from train_heading import evaluate,brief
from select_progress import stress
from reachy_locomotion.heading import HeadingSensor


def fresh_domains(base,seed,count):
    rng=np.random.default_rng(seed);cases=[]
    for i in range(count):
        c=copy.deepcopy(base)
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
        c['foot_radius_m']=float(rng.uniform(.054,.058))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6)),pid_scale=float(rng.uniform(.85,1.15)),
            delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
            passive_friction_nm=float(rng.uniform(.0007,.002)),joint_friction_nm=float(rng.uniform(.007,.015)),
            joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
        cases.append(c)
    return cases


def paired(configs):
    return [dict(copy.deepcopy(c),timestep_s=dt) for c in configs for dt in [.002,.001]]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--search',action='append',required=True)
    p.add_argument('--output',required=True);p.add_argument('--top',type=int,default=6)
    p.add_argument('--retain-speed',action='store_true',help='Select among candidates retaining >=14 mm/s nominal speed')
    p.add_argument('--heldout-seed',type=int,default=2026111905)
    p.add_argument('--freeze-only',action='store_true');a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    base=json.loads(Path('configs/motors-datasheet.json').read_text());sensor=asdict(HeadingSensor())
    archive=[]
    for f in a.search:archive+=json.loads(Path(f).read_text())['history']
    if a.retain_speed:
        archive=[o for o in archive if o['metrics'][0]['speed_mm_s']>=14.]
    archive.sort(key=lambda o:o['score'],reverse=True)
    unique={}
    for o in archive:unique.setdefault(json.dumps([o['gait'],o['feedback']],sort_keys=True),o)
    candidates=list(unique.values())[:a.top]
    receipt=dict(evidence='uncalibrated_simulation',hardware_ready=False,search=a.search,
                 retention_requested=a.retain_speed,
                 development_seed=591191,heldout_seed=a.heldout_seed,sensor=sensor,
                 source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                     [Path(__file__),Path('experiments/train_heading.py'),Path('src/reachy_locomotion/heading.py'),
                      Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/motors.py')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        def ranking(o):
            retained=all(m['speed_mm_s']>=14 and m['feasible'] for m in o['metrics'][:2])
            no_stall=all(m['speed_mm_s']>=8 for m in o['metrics'])
            return (retained if a.retain_speed else True,no_stall if a.retain_speed else True,o['feasible_all'],o['score'])
        valid=list(pool.map(evaluate,[(o['gait'],o['feedback'],paired([base]+stress(base)),12.,sensor) for o in candidates]))
        valid.sort(key=ranking,reverse=True);receipt['development_12s']=valid
        path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
        for o in valid:print('12s',json.dumps(brief(o)),flush=True)
        long=list(pool.map(evaluate,[(o['gait'],o['feedback'],paired([base]+stress(base)),20.,sensor) for o in valid[:3]]))
        long.sort(key=ranking,reverse=True);receipt['development_20s']=long
        chosen=long[0];receipt['frozen_policy']=dict(gait=chosen['gait'],feedback=chosen['feedback'])
        path.with_name(path.stem+'-policy.json').write_text(json.dumps(receipt['frozen_policy'],indent=2)+'\n')
        path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
        print('FROZEN',json.dumps(brief(chosen)),flush=True)
        if not a.freeze_only:
            fresh=paired(fresh_domains(base,receipt['heldout_seed'],10));receipt['fresh_configs']=fresh
            baseline=json.loads(Path('configs/sway-progress-fast.json').read_text())
            tasks=[(label,g,fb,c,dict(sensor,seed=191071+7919*(i//2)))
                   for label,g,fb in [('feedback',chosen['gait'],chosen['feedback']),('open_loop',baseline,None)]
                   for i,c in enumerate(fresh)]
            out=list(pool.map(evaluate,[(g,fb,[c],12.,s) for _,g,fb,c,s in tasks]))
            receipt['heldout']=[dict(label=l,trial=o) for (l,g,fb,c,s),o in zip(tasks,out)]
            variants=[dict(delay_frames=0,noise_std_deg=0,bias_deg=0,seed=19),
                      dict(delay_frames=5,noise_std_deg=.5,bias_deg=1.5,seed=20),
                      dict(delay_frames=10,noise_std_deg=1,bias_deg=-2,seed=21)]
            tasks=[(si,i,c,s) for si,s in enumerate(variants) for i,c in enumerate(paired([base,stress(base)[0],stress(base)[3]]))]
            out=list(pool.map(evaluate,[(chosen['gait'],chosen['feedback'],[c],12.,s) for si,i,c,s in tasks]))
            receipt['sensor_stress']=[dict(sensor_index=si,case_index=i,sensor=s,trial=o) for (si,i,c,s),o in zip(tasks,out)]
    receipt['complete']=True;path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    print('Completed',path,flush=True)
if __name__=='__main__':main()
