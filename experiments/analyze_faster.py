"""Compare the frozen faster policy with the prior gait and matched ablations."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from select_speed import stress_job
from train_faster import read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selection',required=True)
    p.add_argument('--recording',required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args(); out=Path(args.output)
    if out.exists(): p.error('Choose a new output path')
    selected=read(args.selection)
    if not selected['completed']: p.error('Wait for selection to finish')
    recipe=selected['frozen_recipe']; nominal=selected['nominal']
    previous=next(c for c in selected['development'] if c['id']=='previous')
    fine=next(c for c in selected['fine'] if c['id']==recipe['id'])
    endurance=next(c for c in selected['endurance'] if c['id']==recipe['id'])
    stress=selected['stress']; oldg=read('configs/sway-speed-trained.json')
    jobs=[('selected',recipe,read('configs/motors-speed-trained.json'),10.)]
    for label,keys in [('old_yaw_amplitude',['yaw_deg']),('old_sharpness',['yaw_sharpness','roll_sharpness']),
                       ('old_timing',['yaw_phase_deg','phase_skew']),('no_harmonic',['yaw_harmonic_deg'])]:
        r=copy.deepcopy(recipe)
        for key in keys: r['gait'][key]=oldg.get(key,0.)
        jobs.append((label,r,read('configs/motors-speed-trained.json'),10.))
    jobs.append(('endurance_120s',recipe,read('configs/motors-speed-trained.json'),120.))
    fine_config=read('configs/motors-speed-trained.json'); fine_config['timestep_s']=.00025
    jobs.append(('fine_0.25ms',recipe,fine_config,20.))
    recording=read(args.recording)
    rows=[r for r in recording['trajectory'] if r['measuring']]
    forces=np.array([r['foot_side_normal_forces_n_right_left'] for r in rows])
    edge=((forces[:,0]<.1)&(forces[:,1]>.1))|((forces[:,1]<.1)&(forces[:,0]>.1))
    rpy=np.unwrap(np.array([r['head_rpy_rad'] for r in rows]),axis=0)
    rate=np.gradient(rpy,.02,axis=0)
    yaw_slowing=(np.abs(rate[:,2])>.3)&(np.gradient(np.abs(rate[:,2]),.02)<-1.)
    old_demo=read('experiments/reports/motor-training-speed-demo-2026-11-19.json')
    yaw_span=np.ptp(selected['motion_diagnostics']['head_yaw_range_deg'])
    old_yaw_span=np.ptp(old_demo['diagnostics']['head_yaw_range_deg'])
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                nominal_metrics=selected['nominal_metrics'],previous_metrics=previous['metrics'],
                speed_ratio_to_previous=selected['nominal_metrics']['speed_mm_s']/previous['metrics']['speed_mm_s'],
                forward_m=nominal['measurement_displacement_in_startup_frame_m'][0],
                lateral_m=nominal['measurement_displacement_in_startup_frame_m'][1],
                measured_seconds=nominal['measured_seconds'],
                fine_timestep_metrics=fine['metrics'],fine_timestep_relative_difference=fine['relative_difference_from_1ms'],
                endurance_metrics=endurance['metrics'],endurance_seconds=endurance['result']['measured_seconds'],
                endurance_forward_m=endurance['result']['measurement_displacement_in_startup_frame_m'][0],
                max_foot_tilt_deg=endurance['result']['max_foot_tilt_deg'],
                min_joint_limit_margin_rad=endurance['result']['min_motor_joint_limit_margin_rad'],
                fresh_stable_count=sum(c['metrics']['stable'] for c in stress),
                fresh_heading_pass_count=sum(c['metrics']['feasible'] for c in stress),
                fresh_count=len(stress),fresh_max_heading_deg=max(c['metrics']['heading_deg'] for c in stress),
                fresh_speed_range_mm_s=[min(c['metrics']['speed_mm_s'] for c in stress),max(c['metrics']['speed_mm_s'] for c in stress)],
                motion_diagnostics=selected['motion_diagnostics'],
                previous_motion_diagnostics=old_demo['diagnostics'],
                actual_yaw_span_ratio=float(yaw_span/old_yaw_span),
                peak_observed_yaw_slowing_deg_s2=float(np.degrees(np.max(-np.gradient(np.abs(rate[:,2]),.02)))),
                negative_motor_work_per_second_w=nominal['negative_motor_work_j']/nominal['measured_seconds'],
                previous_negative_motor_work_per_second_w=old_demo['nominal']['negative_motor_work_j']/old_demo['nominal']['measured_seconds'],
                single_edge_fraction_50hz=float(edge.mean()),
                yaw_slowing_on_edge_fraction=float(edge[yaw_slowing].mean()) if yaw_slowing.any() else None,
                phase_correlation_is_not_causal_proof=True,
                physical_and_software_motor_limits_unchanged=True,
                motor_config_sha256=hashlib.sha256(Path('configs/motors-speed-trained.json').read_bytes()).hexdigest(),
                ablations=[],extra_validation=[])
    def save(): out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(stress_job,jobs):
            report['extra_validation' if r['case'] in ['endurance_120s','fine_0.25ms'] else 'ablations'].append(r);save()
            print(json.dumps(dict(case=r['case'],**r['metrics'])),flush=True)
    report['completed']=True; save()
    print(json.dumps({k:v for k,v in report.items() if k not in ['ablations','extra_validation']}),flush=True)


if __name__=='__main__': main()
