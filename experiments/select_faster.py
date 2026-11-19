"""Long/fine validation of the larger-swing search, then frozen stress evaluation."""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator
from select_speed import stress_job
from train_faster import evaluate, flatten, read
from train_speed import metrics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', nargs='+', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--recording', required=True)
    args = p.parse_args()
    output, recording = Path(args.output), Path(args.recording)
    if output.exists() or recording.exists():
        p.error('Preserve prior evidence; choose new paths')
    sources = [read(path) for path in args.input]
    if not all(r['completed'] for r in sources):
        p.error('Wait for all search runs to finish')
    all_rows = [c for r in sources for c in flatten(r) if c['metrics']['stable'] and c['metrics']['heading_deg'] <= 35]
    all_rows.sort(key=lambda c:c['metrics']['speed_mm_s'],reverse=True)
    unique = {}
    for c in all_rows:
        unique.setdefault(json.dumps(c['gait'],sort_keys=True),c)
    ranked = list(unique.values())
    shortlist = ranked[:16]
    big = [c for c in ranked if c['gait']['yaw_deg'] >= 42 and c['metrics']['speed_mm_s'] > 35]
    shortlist += big[:2]
    shortlist = list({json.dumps(c['gait'],sort_keys=True):c for c in shortlist}.values())
    feedback = read('configs/heading-speed-trained.json')
    variants = {'same':feedback,
                'earlier':dict(feedback,heading_deadband_deg=5),
                'stronger':dict(feedback,heading_deadband_deg=7,kp=feedback['kp']*1.4,ki=feedback['ki']*.7)}
    jobs = [(f'{i}-{name}',c['gait'],fb,20.,.001) for i,c in enumerate(shortlist) for name,fb in variants.items()]
    jobs.append(('previous',read('configs/sway-speed-trained.json'),feedback,20.,.001))
    report = dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                  args=vars(args),physical_and_software_motor_limits_unchanged=True,
                  development=[],fine=[],endurance=[],stress=[],stress_seed=2026111923,
                  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                                 [Path(__file__),Path('experiments/train_faster.py'),
                                  Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py'),
                                  Path('src/reachy_locomotion/motors.py'),Path('configs/motors-speed-trained.json')]})
    output.parent.mkdir(parents=True,exist_ok=True)
    def save():
        output.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(evaluate,jobs):
            report['development'].append(r); save()
            print(json.dumps(dict(stage='20s',id=r['id'],**r['metrics'])),flush=True)
        eligible = sorted([r for r in report['development'] if r['metrics']['feasible']],key=lambda r:r['metrics']['speed_mm_s'],reverse=True)
        if not eligible:
            raise RuntimeError('No candidate meets the 20-degree/no-fall constraints')
        shortlist = eligible[:8]
        jobs = [(r['id'],r['gait'],r['feedback'],20.,.0005) for r in shortlist]
        for old,r in zip(shortlist,pool.map(evaluate,jobs)):
            delta=abs(r['metrics']['speed_mm_s']-old['metrics']['speed_mm_s'])/max(abs(r['metrics']['speed_mm_s']),abs(old['metrics']['speed_mm_s']),1e-9)
            r['relative_difference_from_1ms']=delta
            r['accepted']=r['metrics']['feasible'] and delta<=.10
            r['conservative_speed_mm_s']=min(old['metrics']['speed_mm_s'],r['metrics']['speed_mm_s'])
            report['fine'].append(r); save()
            print(json.dumps(dict(stage='0.5ms',id=r['id'],difference=delta,**r['metrics'])),flush=True)
        shortlist=sorted([r for r in report['fine'] if r['accepted']],key=lambda r:r['conservative_speed_mm_s'],reverse=True)[:4]
        jobs=[(r['id'],r['gait'],r['feedback'],40.,.001) for r in shortlist]
        for fine,r in zip(shortlist,pool.map(evaluate,jobs)):
            r['selection_speed_mm_s']=min(fine['conservative_speed_mm_s'],r['metrics']['speed_mm_s'])
            report['endurance'].append(r); save()
            print(json.dumps(dict(stage='40s',id=r['id'],**r['metrics'])),flush=True)
        survivors=[r for r in report['endurance'] if r['metrics']['feasible']]
        if not survivors:
            raise RuntimeError('No candidate survives long/fine validation')
        best=max(survivors,key=lambda r:r['selection_speed_mm_s'])
        recipe=dict(id=best['id'],gait=best['gait'],feedback=best['feedback'])
        report['frozen_recipe']=recipe; save()
        # Fresh model trials are not consulted when choosing this policy.
        rng=np.random.default_rng(report['stress_seed']); jobs=[]
        for i in range(8):
            c=read('configs/motors-speed-trained.json')
            c['foot_radius_m']=float(rng.uniform(.054,.058))
            c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
            c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6)),pid_scale=float(rng.uniform(.85,1.15)),
                                   delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
                                   passive_friction_nm=float(rng.uniform(.0007,.002)),joint_friction_nm=float(rng.uniform(.007,.015)),
                                   joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
            for dt in [.001,.0005]:
                variant=copy.deepcopy(c); variant['timestep_s']=dt
                jobs.append((f'fresh-{i}-{dt}',recipe,variant,20.))
        for r in pool.map(stress_job,jobs):
            report['stress'].append(r); save()
            print(json.dumps(dict(stage='fresh',case=r['case'],**r['metrics'])),flush=True)
    config=read('configs/motors-speed-trained.json')
    g=Gait(**recipe['gait']); period=round(50/g.frequency_hz)/50
    sim=SwaySimulator('experiments/upstream-reachy-mini',config)
    r=sim.run(g,cycles=math.ceil(20/period-1e-10),warmup_cycles=3,record=True,
              feedback=recipe['feedback'],sensor=read('configs/heading-sensor-assumed.json'))
    expected=next(c['result'] for c in report['development'] if c['id']==recipe['id'])
    for k in ['forward_m','measurement_displacement_in_startup_frame_m','max_heading_from_startup_rad','negative_motor_work_j']:
        assert r[k]==expected[k],(k,r[k],expected[k])
    r['render_forward_frame']='startup'
    recording.parent.mkdir(parents=True,exist_ok=True)
    recording.write_text(json.dumps(r,allow_nan=False)+'\n')
    Path('configs/sway-faster-trained.json').write_text(json.dumps(recipe['gait'],indent=2)+'\n')
    Path('configs/heading-faster-trained.json').write_text(json.dumps(recipe['feedback'],indent=2)+'\n')
    rows=[x for x in r['trajectory'] if x['measuring']]
    rpy=np.unwrap(np.array([x['head_rpy_rad'] for x in rows]),axis=0)
    rates=np.gradient(rpy,.02,axis=0)
    report['recorded_replay_matches_selection_exactly']=True
    report['recording_sha256']=hashlib.sha256(recording.read_bytes()).hexdigest()
    report['nominal']={k:v for k,v in r.items() if k!='trajectory'}
    report['nominal_metrics']=metrics(r)
    report['motion_diagnostics']=dict(head_roll_range_deg=np.degrees([rpy[:,0].min(),rpy[:,0].max()]).tolist(),
                                    head_yaw_range_deg=np.degrees([rpy[:,2].min(),rpy[:,2].max()]).tolist(),
                                    peak_roll_yaw_rate_deg_s=np.degrees(np.max(np.abs(rates[:,[0,2]]),axis=0)).tolist(),
                                    negative_motor_work_j=r['negative_motor_work_j'],airborne_fraction=r['airborne_fraction'])
    report['completed']=True; save()
    print(json.dumps(dict(stage='recorded',**report['nominal_metrics'],diagnostics=report['motion_diagnostics'])),flush=True)


if __name__=='__main__':
    main()
