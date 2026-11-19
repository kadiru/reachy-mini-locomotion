"""Validate and freeze a distinct large-rocking gait, then evaluate fresh models."""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait,SwaySimulator
from reachy_locomotion.rocking import rocking_metrics
from train_rocking import evaluate,feedback,flatten,read


def stress_job(job):
    label,recipe,config,seconds=job
    sim=SwaySimulator('experiments/upstream-reachy-mini',config);g=Gait(**recipe['gait'])
    period=round(50/g.frequency_hz)/50
    r=sim.run(g,cycles=math.ceil(seconds/period-1e-10),warmup_cycles=3,feedback=recipe['feedback'],
              sensor=read('configs/heading-sensor-assumed.json') if recipe['feedback'] else None)
    return dict(id=label,metrics=rocking_metrics(r),result=r)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',nargs='+',required=True);p.add_argument('--output',required=True)
    p.add_argument('--recording',required=True)
    p.add_argument('--recipe-mode',action='store_true',help='Keep each learned gait/feedback pair together')
    p.add_argument('--allow-style-near-miss',action='store_true',help='Explicitly retain a provisional larger-than-rapid gait; original style metrics still report failure')
    p.add_argument('--stress-seed',type=int,default=2026111933)
    args=p.parse_args()
    out=Path(args.output);recording=Path(args.recording)
    if out.exists() or recording.exists():p.error('Preserve prior evidence')
    sources=[read(x) for x in args.input]
    if not all(x['completed'] for x in sources):p.error('Wait for searches to finish')
    rapid_stride=read('experiments/reports/motor-training-gait-styles-baseline-2026-11-19.json')['mean_forward_mm_per_left_right_cycle']
    def acceptable(m):
        if not args.allow_style_near_miss:return m['feasible']
        return m['course_ok'] and m['bilateral_roll_deg']>=6 and m['both_edges_cycle_fraction']>=.8 and m['stride_mm']>rapid_stride
    candidates=sorted([c for r in sources for c in flatten(r) if c['metrics']['stable']],key=lambda c:c['metrics']['score'],reverse=True)
    unique={}
    for c in candidates:unique.setdefault(json.dumps([c['gait'],c['feedback']] if args.recipe_mode else c['gait'],sort_keys=True),c)
    shortlist=list(unique.values())[:16]
    variants={'open':None,'endpoint':feedback(),
              'strong':dict(feedback(),kp=.18,ki=.025,max_correction_deg=5,heading_deadband_deg=3),
              'yaw_only':dict(feedback(),roll_per_correction=0,yaw_per_roll=-2,phase_skew_per_correction=0),
              'reverse_yaw':dict(feedback(),roll_per_correction=0,yaw_per_roll=2,phase_skew_per_correction=0)}
    jobs=[(f'{i}-{name}',c['gait'],fb,20.,.001) for i,c in enumerate(shortlist) for name,fb in variants.items()]
    if args.recipe_mode:
        jobs=[(f'{i}-learned',c['gait'],c['feedback'],20.,.001) for i,c in enumerate(shortlist)]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                heading_rule='<=20 deg relative to startup after every complete cycle; instantaneous yaw is diagnostic',
                development=[],fine=[],endurance=[],stress=[],stress_seed=args.stress_seed,
                acceptance_mode='provisional style near-miss; original 15mm target still reported' if args.allow_style_near_miss else 'original full style gates',
                source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/rocking.py'),Path('src/reachy_locomotion/heading.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(evaluate,jobs):
            report['development'].append(r);save();print(json.dumps(dict(stage='20s',id=r['id'],**r['metrics'])),flush=True)
        eligible=sorted([r for r in report['development'] if acceptable(r['metrics'])],key=lambda r:r['metrics']['stride_mm'],reverse=True)
        if not eligible:raise RuntimeError('No candidate meets long-run heading, rocking and stride requirements')
        shortlist=eligible[:8]
        for old,r in zip(shortlist,pool.map(evaluate,[(x['id'],x['gait'],x['feedback'],20.,.0005) for x in shortlist])):
            delta=abs(old['metrics']['stride_mm']-r['metrics']['stride_mm'])/max(abs(old['metrics']['stride_mm']),abs(r['metrics']['stride_mm']),1e-9)
            r['relative_stride_difference']=delta;r['accepted']=acceptable(r['metrics']) and delta<=.15
            r['conservative_stride_mm']=min(old['metrics']['stride_mm'],r['metrics']['stride_mm'])
            report['fine'].append(r);save();print(json.dumps(dict(stage='0.5ms',id=r['id'],difference=delta,**r['metrics'])),flush=True)
        shortlist=sorted([r for r in report['fine'] if r['accepted']],key=lambda r:r['conservative_stride_mm'],reverse=True)[:4]
        for fine,r in zip(shortlist,pool.map(evaluate,[(x['id'],x['gait'],x['feedback'],40.,.001) for x in shortlist])):
            r['selection_stride_mm']=min(fine['conservative_stride_mm'],r['metrics']['stride_mm'])
            report['endurance'].append(r);save();print(json.dumps(dict(stage='40s',id=r['id'],**r['metrics'])),flush=True)
        survivors=[r for r in report['endurance'] if acceptable(r['metrics'])]
        if not survivors:raise RuntimeError('No candidate passes the long/fine style gates')
        best=max(survivors,key=lambda r:r['selection_stride_mm'])
        recipe=dict(id=best['id'],gait=best['gait'],feedback=best['feedback'])
        report['frozen_recipe']=recipe;save()
        rng=np.random.default_rng(report['stress_seed']);jobs=[]
        for i in range(8):
            c=read('configs/motors-speed-trained.json');c['foot_radius_m']=float(rng.uniform(.054,.058))
            c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
            c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6)),pid_scale=float(rng.uniform(.85,1.15)),
                                   delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
                                   passive_friction_nm=float(rng.uniform(.0007,.002)),joint_friction_nm=float(rng.uniform(.007,.015)),
                                   joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
            for dt in [.001,.0005]:
                variant=copy.deepcopy(c);variant['timestep_s']=dt;jobs.append((f'fresh-{i}-{dt}',recipe,variant,20.))
        for r in pool.map(stress_job,jobs):
            report['stress'].append(r);save();print(json.dumps(dict(stage='fresh',id=r['id'],**r['metrics'])),flush=True)
    g=Gait(**recipe['gait']);period=round(50/g.frequency_hz)/50
    config=read('configs/motors-speed-trained.json');sim=SwaySimulator('experiments/upstream-reachy-mini',config)
    r=sim.run(g,cycles=math.ceil(20/period-1e-10),warmup_cycles=3,record=True,feedback=recipe['feedback'],
              sensor=read('configs/heading-sensor-assumed.json') if recipe['feedback'] else None)
    expected=next(c['result'] for c in report['development'] if c['id']==recipe['id'])
    for k in ['measurement_displacement_in_startup_frame_m','rocking_cycles','max_foot_tilt_deg','negative_motor_work_j']:
        assert r[k]==expected[k],k
    r.update(render_forward_frame='startup',evaluation_heading_rule='completed_cycle')
    recording.write_text(json.dumps(r,allow_nan=False)+'\n')
    Path('configs/sway-rocking-trained.json').write_text(json.dumps(recipe['gait'],indent=2)+'\n')
    Path('configs/heading-rocking-trained.json').write_text(json.dumps(recipe['feedback'],indent=2)+'\n')
    report['nominal']={k:v for k,v in r.items() if k!='trajectory'};report['nominal_metrics']=rocking_metrics(r)
    report['recorded_replay_matches_selection_exactly']=True
    report['recording_sha256']=hashlib.sha256(recording.read_bytes()).hexdigest()
    report['completed']=True;save();print(json.dumps(dict(stage='recorded',**report['nominal_metrics'])),flush=True)


if __name__=='__main__':main()
