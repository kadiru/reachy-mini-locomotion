"""Train the separate large-rocking style under unchanged motor limits."""
import argparse
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator
from reachy_locomotion.rocking import rocking_metrics


KEYS=['frequency_hz','roll_deg','yaw_deg','yaw_phase_deg','pitch_bias_deg','x_mm','y_mm','y_phase_deg',
      'z_mm','yaw_sharpness','roll_sharpness','yaw_bias_deg','roll_bias_deg','phase_skew','pitch_deg',
      'pitch_phase_deg','yaw_stroke_fraction','roll_stroke_fraction','stroke_accel_fraction','stroke_brake_fraction',
      'yaw_harmonic_deg','yaw_harmonic_phase_deg']
LOW=np.array([.6,20,10,0,-12,-24,-24,-180,-12,0,0,-8,-4,-.5,-12,-180,0,0,.025,.025,-8,-180])
HIGH=np.array([3.5,40,60,180,18,16,24,180,8,4,4,8,4,.5,12,180,1,1,.45,.35,8,180])
CACHE={}


def read(path):return json.loads(Path(path).read_text())


def vector(g):
    g=asdict(Gait(**g));return np.array([g[k] for k in KEYS])


def decode(v):
    g=dict(zip(KEYS,np.clip(v,LOW,HIGH).tolist()))
    g['roll_deg']=min(g['roll_deg'],40-abs(g['roll_bias_deg']))
    g['yaw_deg']=min(g['yaw_deg'],60-abs(g['yaw_bias_deg'])-abs(g['yaw_harmonic_deg']))
    for key in ['yaw_stroke_fraction','roll_stroke_fraction']:
        if g[key]<.15:g[key]=0.
    return g


def feedback(kind='endpoint'):
    if kind=='open':return None
    return dict(kp=.08,ki=.015,kd=0,filter_tau_s=.15,max_correction_deg=4,
                correction_rate_deg_s=12,yaw_per_roll=-.25,roll_per_correction=1,
                phase_skew_per_correction=-.015,cycle_endpoint=True,heading_deadband_deg=5,
                startup_ramp_cycles=2)


def evaluate(job):
    name,g,fb,seconds,dt=job
    config=read('configs/motors-speed-trained.json');config['timestep_s']=dt
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=SwaySimulator('experiments/upstream-reachy-mini',config)
    gait=Gait(**g);period=round(50/gait.frequency_hz)/50
    r=CACHE[key].run(gait,cycles=math.ceil(seconds/period-1e-10),warmup_cycles=3,
                     feedback=fb,sensor=read('configs/heading-sensor-assumed.json') if fb else None)
    return dict(id=name,gait=g,feedback=fb,metrics=rocking_metrics(r),result=r)


def seeds():
    candidates=[]
    for name in ['speed-existing','speed-sharp','faster-braking']:
        r=read(f'experiments/reports/motor-training-{name}-2026-11-19.json')
        for c in r.get('candidates',[])+r.get('initial',[])+[c for h in r.get('history',[]) for c in h['candidates']]:
            x=c['result']
            if x['valid'] and x['forward_m_per_s']>.002:
                candidates.append(c)
    candidates.sort(key=lambda c:c['result']['max_foot_tilt_deg'],reverse=True)
    unique={}
    for c in candidates:unique.setdefault(json.dumps(c['gait'],sort_keys=True),c)
    gaits=[decode(vector(c['gait'])) for c in list(unique.values())[:24]]
    rng=np.random.default_rng(2026111930)
    for i in range(40):
        g=dict(frequency_hz=float(rng.uniform(.8,3)),roll_deg=float(rng.uniform(28,40)),
               yaw_deg=float(rng.uniform(20,50)),yaw_phase_deg=float(rng.uniform(45,140)),
               pitch_bias_deg=float(rng.uniform(-5,12)),x_mm=float(rng.uniform(-18,6)),
               y_mm=float(rng.uniform(-20,20)),y_phase_deg=float(rng.uniform(-180,180)),
               z_mm=float(rng.uniform(-6,3)),roll_sharpness=float(rng.uniform(0,3)),
               yaw_sharpness=float(rng.uniform(0,3)),phase_skew=float(rng.uniform(-.3,.3)))
        if i%2:g.update(yaw_stroke_fraction=float(rng.uniform(.3,.95)),stroke_brake_fraction=.03)
        gaits.append(decode(vector(g)))
    return gaits


def flatten(r):return r.get('candidates',[])+[c for h in r.get('history',[]) for c in h['candidates']]


def ranking(candidate,progress_focus):
    m=candidate['metrics']
    if not progress_focus:return m['score']
    if not m['stable']:return -1e6
    # Once a large rocking basin exists, stop rewarding extra tilt above the
    # six-degree style threshold and concentrate exploration on longer strides.
    return ((1000 if m['feasible'] else 0)+m['stride_mm']+min(m['bilateral_roll_deg'],6)
            -3*max(0.,6-m['bilateral_roll_deg'])-4*max(0.,m['cycle_heading_deg']-20)
            -40*max(0.,.8-m['both_edges_cycle_fraction']))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['probe','search'],default='probe')
    p.add_argument('--input',nargs='+');p.add_argument('--output',required=True)
    p.add_argument('--progress-focus',action='store_true')
    p.add_argument('--local-scale',type=float,default=1.)
    p.add_argument('--generations',type=int,default=8);p.add_argument('--population',type=int,default=36)
    p.add_argument('--seed',type=int,default=2026111931)
    p.add_argument('--seconds',type=float,default=8)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Choose a new report')
    if not 1<=args.generations<=16 or not 12<=args.population<=64 or not 4<=args.seconds<=20:p.error('Use a bounded CPU search')
    if not .2<=args.local_scale<=1:p.error('Local mutation scale must be in [.2,1]')
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                objective='larger signed progress per full left/right cycle, bilateral rocking and support transfer; <=20 degrees at every completed cycle',
                style_targets=dict(median_bilateral_roll_deg=6,mean_stride_mm=15,both_edges_cycle_fraction=.8),
                physical_and_software_motor_limits_unchanged=True,candidates=[],history=[],
                source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/rocking.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    rng=np.random.default_rng(args.seed)
    with ProcessPoolExecutor(max_workers=4) as pool:
        if args.mode=='probe':
            jobs=[(f'seed-{i}-{kind}',g,feedback(kind),args.seconds,.001) for i,g in enumerate(seeds()) for kind in ['open','endpoint']]
            jobs.append(('rapid_baseline',read('configs/sway-faster-trained.json'),read('configs/heading-faster-trained.json'),args.seconds,.001))
            for r in pool.map(evaluate,jobs):
                report['candidates'].append(r);save();print(json.dumps(dict(id=r['id'],**r['metrics'])),flush=True)
        else:
            previous=[read(path) for path in args.input]
            if not all(r['completed'] for r in previous):p.error('Wait for seed report completion')
            rank=lambda c:ranking(c,args.progress_focus)
            ranked=sorted([c for r in previous for c in flatten(r)],key=rank,reverse=True)
            unique={}
            for c in ranked:unique.setdefault(json.dumps([c['gait'],c['feedback']],sort_keys=True),c)
            anchors=list(unique.values())[:16]
            if any(abs(c['result']['measured_seconds']-args.seconds) > 1/c['result']['actual_frequency_hz']+.001 for c in anchors):
                anchors=list(pool.map(evaluate,[(f'reseed-{i}',c['gait'],c['feedback'],args.seconds,.001) for i,c in enumerate(anchors)]))
                anchors.sort(key=rank,reverse=True);report['initial']=anchors;save()
            best=anchors[0];mean=vector(best['gait']);std=.065*args.local_scale*(HIGH-LOW)
            for gen in range(args.generations):
                rows=rng.normal(mean,std,size=(args.population,len(KEYS)));fbs=[]
                for i in range(args.population):
                    anchor=best if i%2 else anchors[(gen+i)%len(anchors)]
                    if i==0:rows[i]=vector(best['gait'])
                    elif i%3:rows[i]=rng.normal(vector(anchor['gait']),args.local_scale*(.025 if i%4==1 else .065)*(HIGH-LOW))
                    fbs.append(anchor['feedback'])
                candidates=list(pool.map(evaluate,[(f'g{gen}-{i}',decode(row),fbs[i],args.seconds,.001) for i,row in enumerate(rows)]))
                candidates.sort(key=rank,reverse=True)
                if rank(candidates[0])>rank(best):best=candidates[0]
                elites=np.array([vector(c['gait']) for c in candidates[:6]])
                mean=.25*mean+.75*elites.mean(axis=0);std=np.maximum(.25*std+.75*elites.std(axis=0),.012*args.local_scale*(HIGH-LOW))
                anchors=sorted(anchors+candidates[:5],key=rank,reverse=True)[:16]
                report['history'].append(dict(generation=gen,candidates=candidates));report['best_short']=best;save()
                print(json.dumps(dict(generation=gen,search_score=rank(best),**best['metrics'],gait=best['gait'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
