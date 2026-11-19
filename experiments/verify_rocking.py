"""Additional frozen-policy checks and compact evidence for the rocking mode."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from select_rocking import stress_job
from train_rocking import read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selection',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Choose a new report')
    s=read(args.selection)
    if not s['completed']:p.error('Wait for completed selection')
    recipe=s['frozen_recipe'];r=s['nominal'];m=s['nominal_metrics']
    fine=next(c for c in s['fine'] if c['id']==recipe['id'])
    long=next(c for c in s['endurance'] if c['id']==recipe['id'])
    previous=read('experiments/reports/motor-training-gait-styles-baseline-2026-11-19.json')
    stress=s['stress'];config=read('configs/motors-speed-trained.json')
    finer=copy.deepcopy(config);finer['timestep_s']=.00025
    mirror=copy.deepcopy(recipe);mirror['gait']['mirror']=-1
    no_brake=copy.deepcopy(recipe);no_brake['gait']['yaw_stroke_fraction']=0
    no_brake['gait']['yaw_sharpness']=0
    jobs=[('120s',recipe,config,120.),('0.25ms',recipe,finer,20.),
          ('mirrored',mirror,config,20.),('sine_yaw_ablation',no_brake,config,20.)]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                acceptance_mode=s.get('acceptance_mode','original full style gates'),
                original_style_gates_pass_all_selection_checks=all(x['feasible'] for x in [m,fine['metrics'],long['metrics']]),
                nominal_metrics=m,forward_m=r['measurement_displacement_in_startup_frame_m'][0],
                lateral_m=r['measurement_displacement_in_startup_frame_m'][1],measured_seconds=r['measured_seconds'],
                actual_frequency_hz=r['actual_frequency_hz'],max_body_tilt_deg=r['max_foot_tilt_deg'],
                minimum_joint_margin_rad=r['min_motor_joint_limit_margin_rad'],
                max_rim_height_mm=r['max_side_rim_clearance_m']*1000,
                mean_within_cycle_yaw_excursion_deg=r['rocking_cycles']['mean_yaw_excursion_deg'],
                min_cycle_progress_mm=r['rocking_cycles']['min_forward_mm_per_cycle'],
                airborne_fraction=r['airborne_fraction'],
                stride_ratio_to_rapid=m['stride_mm']/previous['mean_forward_mm_per_left_right_cycle'],
                max_tilt_ratio_to_rapid=r['max_foot_tilt_deg']/previous['max_body_tilt_deg'],
                fine_metrics=fine['metrics'],fine_relative_stride_difference=fine['relative_stride_difference'],
                endurance_40s_metrics=long['metrics'],
                fresh_count=len(stress),fresh_stable_count=sum(c['metrics']['stable'] for c in stress),
                fresh_course_pass_count=sum(c['metrics']['course_ok'] for c in stress),
                fresh_full_style_pass_count=sum(c['metrics']['feasible'] for c in stress),
                fresh_failure_reasons={c['id']:c['result']['failure_reason'] for c in stress if not c['metrics']['stable']},
                fresh_max_cycle_heading_deg=max(c['metrics']['cycle_heading_deg'] for c in stress),
                source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/rocking.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py'),Path('src/reachy_locomotion/motors.py'),Path('configs/motors-speed-trained.json')]},
                extra_checks=[])
    def save():out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(stress_job,jobs):
            report['extra_checks'].append(c);save();print(json.dumps(dict(id=c['id'],**c['metrics'])),flush=True)
    report['completed']=True;save();print(json.dumps({k:v for k,v in report.items() if k!='extra_checks'}),flush=True)


if __name__=='__main__':main()
