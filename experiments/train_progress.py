"""Progress-first CEM with turn/drift constraints and finite swing-and-hold strokes.

Nominal exploration is followed by model/timestep validation; only development
conditions select the policy. Unseen domains are evaluated after it is frozen.
"""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import SwaySimulator,Gait

KEYS=['frequency_hz','roll_deg','yaw_deg','yaw_phase_deg','pitch_bias_deg','x_mm','y_mm','y_phase_deg','z_mm','yaw_sharpness','roll_sharpness','yaw_bias_deg','roll_bias_deg']
LOW=np.array([1.1,18,15,-180,-10,-20,-22,-180,-8,0,0,-10,-4.])
HIGH=np.array([3.4,40,60,180,14,15,22,180,8,4.5,3,10,4.])
CACHE={}


def decode(row):
    g=dict(zip(KEYS,np.clip(row,LOW,HIGH).tolist()))
    g['roll_deg']=min(g['roll_deg'],40-abs(g['roll_bias_deg']))
    g['yaw_deg']=min(g['yaw_deg'],60-abs(g['yaw_bias_deg']))
    return g


def vector(g):
    full=asdict(Gait(**g));return np.array([full[k] for k in KEYS])


def metrics(r):
    if not r['valid']:return {'feasible':False,'speed_mm_s':0.,'violation':1e3,'score':-1e4}
    t=r['measured_seconds'];f=r['forward_m'];lat=r['max_lateral_excursion_m']
    yaw=np.degrees(r['max_heading_excursion_rad'])
    # Constraints apply to excursion throughout the run, not just final pose.
    yaw_over=max(0.,yaw-10.)
    lat_over=max(0.,lat-(.3*max(0.,f)+.003))*1000
    tilt_over=max(0.,r['max_foot_tilt_deg']-18.)
    violation=yaw_over+lat_over+2*tilt_over
    speed=1000*f/t
    feasible=violation<1e-9 and speed>=.5
    # Feasible trajectories always rank first. Smoothness has no reward term.
    score=1000+speed if feasible else min(speed,100.)-3*violation
    return {'feasible':bool(feasible),'speed_mm_s':speed,'violation':violation,'score':score}


def evaluate(payload):
    gait,configs,seconds=payload;results=[]
    for c in configs:
        key=json.dumps(c,sort_keys=True)
        if key not in CACHE:CACHE[key]=SwaySimulator('experiments/upstream-reachy-mini',c)
        g=Gait(**gait);period=round(50/g.frequency_hz)/50
        # Whole cycles guarantee the same phase at start/end; speed normalizes
        # the <one-cycle rounding of the measurement duration.
        r=CACHE[key].run(g,cycles=math.ceil(seconds/period),warmup_cycles=3)
        results.append(r)
    mm=[metrics(r) for r in results]
    feasible=all(m['feasible'] for m in mm)
    speed=.65*min(m['speed_mm_s'] for m in mm)+.35*np.mean([m['speed_mm_s'] for m in mm])
    score=1000+speed if feasible else min(speed,100.)-3*max(m['violation'] for m in mm)
    return {'gait':gait,'score':float(score),'feasible_all':feasible,'metrics':mm,'results':results}


def development(base):
    cases=[copy.deepcopy(base) for _ in range(4)]
    cases[1]['timestep_s']=.001
    cases[2]['motor_model']['voltage_v']=5.5
    cases[2]['friction_slide_torsion_roll'][0]=.6
    cases[3]['motor_model'].update(voltage_v=6.,delay_frames=2)
    cases[3]['friction_slide_torsion_roll'][0]=.8
    return cases


def holdout(base):
    rng=np.random.default_rng(491731)
    cases=[]
    for i in range(6):
        c=copy.deepcopy(base);c['foot_radius_m']=float(rng.uniform(.054,.058))
        c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
        c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6.)),
            pid_scale=float(rng.uniform(.85,1.15)),delay_frames=int(rng.integers(0,3)),
            mass_distribution=['torso','uniform'][i%2],
            passive_friction_nm=float(rng.uniform(.0007,.002)),
            joint_friction_nm=float(rng.uniform(.007,.015)),
            joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
        cases.append(c)
    return cases


def cem(pool,seed_gaits,configs,generations,population,seconds,rng,receipt,path,stage):
    seeds=[vector(g) for g in seed_gaits]
    mean=seeds[0].copy();std=.15*(HIGH-LOW)
    best=None;archive=[];stale=0
    for gen in range(generations):
        samples=rng.normal(mean,std,size=(population,len(KEYS)))
        # Keep local search around the best known feasible candidate; restarting
        # avoids losing a narrow advancing basin among stationary elites.
        for i in range(min(len(seeds),population//4)):samples[i]=seeds[i]
        if best:
            bv=vector(best['gait']);samples[0]=bv
            samples[population//2:3*population//4]=rng.normal(bv,.055*(HIGH-LOW),size=(population//4,len(KEYS)))
        samples[-3:]=rng.uniform(LOW,HIGH,size=(3,len(KEYS)))
        candidates=[decode(x) for x in samples]
        out=list(pool.map(evaluate,[(g,configs,seconds) for g in candidates]))
        order=np.argsort([o['score'] for o in out])[::-1]
        if best is None or out[order[0]]['score']>best['score']+1e-8:
            best=out[order[0]];stale=0
        else:stale+=1
        elites=np.array([vector(out[i]['gait']) for i in order[:max(4,population//6)]])
        mean=.2*mean+.8*elites.mean(axis=0)
        std=np.maximum(.3*std+.7*elites.std(axis=0),.015*(HIGH-LOW))
        if stale>=3:
            mean=vector(best['gait']);std=.07*(HIGH-LOW);stale=0
        archive.extend(out)
        receipt['history'].append({'stage':stage,'generation':gen,'candidates':out})
        receipt['current_best']=best
        path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
        print(json.dumps({'stage':stage,'generation':gen,'score':best['score'],
              'feasible':best['feasible_all'],'speed_mm_s':[round(m['speed_mm_s'],3) for m in best['metrics']],
              'heading_deg':[round(np.degrees(r['max_heading_excursion_rad']),2) for r in best['results']]}),flush=True)
    unique={json.dumps(a['gait'],sort_keys=True):a for a in archive}
    return sorted(unique.values(),key=lambda a:a['score'],reverse=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--explore-generations',type=int,default=12);p.add_argument('--refine-generations',type=int,default=8)
    p.add_argument('--population',type=int,default=36);p.add_argument('--seed',type=int,default=2026111903)
    p.add_argument('--seed-policy',action='append',default=[])
    args=p.parse_args();path=Path(args.output)
    if path.exists():p.error('Choose a new report path')
    if not 0<=args.explore_generations<=25 or not 0<=args.refine_generations<=25 or not 12<=args.population<=64:p.error('Bounded local search only')
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    old_fast=json.loads(Path('experiments/reports/motor-training-2026-11-18-policy.json').read_text())
    old_smooth=json.loads(Path('configs/sway-trained.json').read_text())
    larger=dict(old_fast,roll_deg=38,yaw_deg=45,yaw_sharpness=1.5)
    seeds=[old_smooth,old_fast,larger]+[json.loads(Path(f).read_text()) for f in args.seed_policy]
    rng=np.random.default_rng(args.seed)
    receipt={'evidence':'uncalibrated_simulation','hardware_ready':False,'method':'progress_first_constrained_CEM',
             'seed':args.seed,'args':vars(args),'bounds':dict(zip(KEYS,zip(LOW.tolist(),HIGH.tolist()))),
             'development_configs':development(base),'holdout_configs':holdout(base),
             'constraint':'max heading excursion <=10 deg; max lateral <=.3*forward+3mm; tilt<=18deg; valid and speed>=.5mm/s',
             'smoothness_reward':False,'history':[],
             'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                             [Path(__file__),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/motors.py')]}}
    path.parent.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=4) as pool:
        if args.explore_generations:
            archive=cem(pool,seeds,[base],args.explore_generations,args.population,6.6,rng,receipt,path,'explore')
            seeds=[a['gait'] for a in archive[:12]]+seeds
        # Validation is development, not held-out evidence.
        shortlisted=list(pool.map(evaluate,[(g,development(base),10.) for g in seeds]))
        shortlisted.sort(key=lambda a:a['score'],reverse=True);receipt['development_shortlist']=shortlisted
        seeds=[a['gait'] for a in shortlisted[:6]]
        if args.refine_generations:
            archive=cem(pool,seeds,development(base),args.refine_generations,max(24,args.population-12),8.,rng,receipt,path,'robust_refine')
            seeds=[a['gait'] for a in archive[:8]]+seeds
        validation=list(pool.map(evaluate,[(g,development(base),12.) for g in seeds]))
        validation.sort(key=lambda a:a['score'],reverse=True);receipt['long_development_validation']=validation
        best=validation[0];receipt['frozen_policy']=best['gait'];receipt['best_development']=best
        path.with_name(path.stem+'-policy.json').write_text(json.dumps(best['gait'],indent=2)+'\n')
        path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
        # Only now run holdout cases; they never select or tune the policy.
        checks=[]
        for label,g in [('selected',best['gait']),('previous_smooth',old_smooth),('previous_fast',old_fast)]:
            for i,c in enumerate([base]+holdout(base)):
                for dt in [.002,.001]:
                    c=copy.deepcopy(c);c['timestep_s']=dt;checks.append((label,i,g,c))
        out=list(pool.map(evaluate,[(g,[c],12.) for _,_,g,c in checks]))
        receipt['evaluation']=[{'label':l,'domain_index':i,'result':o['results'][0],'metrics':o['metrics'][0]}
                              for (l,i,g,c),o in zip(checks,out)]
    receipt['completed']=True;path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    print('Completed '+str(path),flush=True)


if __name__=='__main__':main()
