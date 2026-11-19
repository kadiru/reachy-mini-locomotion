"""Freeze validated primitives, record matched demonstrations, then probe limits."""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np
from reachy_locomotion.primitives import PrimitiveSimulator
from reachy_locomotion.geometry import euler
from reachy_locomotion.contact_probe import relative_head_pose
from train_primitives import read,summary
from qualify_primitives import pivot_metrics


def check(job):
    label,recipe,waist,config=job
    sim=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    r=sim.run_primitive(recipe,seconds=8,waist_mode=waist);r.pop('trace')
    return dict(id=label,recipe=recipe,result=r,pivot_metrics=pivot_metrics(r))


def record_terminal(sim,r):
    """Append actual terminal qpos, never extrapolate an impact or recovery.

    The terminal frame is kinematic only: force/torque fields are unknown, not
    copied from the preceding sample or recomputed with missing solver state.
    """
    last=r['trajectory'][-1];end=r['terminal_state']
    if end['time_s']<=last['time_s']+1e-8:return
    d=mujoco.MjData(sim.model);d.qpos[:]=end['qpos'];d.qvel[:]=end['qvel'];mujoco.mj_forward(sim.model,d)
    R=d.xmat[sim.base].reshape(3,3);angles=euler(R)
    yaw=last['heading_from_startup_rad']+(angles[2]-last['foot_rpy_rad'][2]+math.pi)%(2*math.pi)-math.pi
    row=dict(last,time_s=end['time_s'],qpos=end['qpos'],qvel=end['qvel'],foot_xyz_m=d.xpos[sim.base].tolist(),
             foot_rpy_rad=angles.tolist(),head_rpy_rad=euler(relative_head_pose(sim.model,d)).tolist(),
             heading_from_startup_rad=float(yaw),side_height_difference_mm=abs(2*sim.config['foot_radius_m']*R[2,1])*1000,
             primitive_stage='fall' if r['fell'] else 'guard stop',terminal_event=r['termination'],
             foot_side_normal_forces_n_right_left=None,waist_torque_nm=None,torso_yaw_deg=None)
    r['trajectory'].append(row)
    r['terminal_frame_note']='Actual terminal kinematics; no invented forces. At most one partial 20ms control interval.'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catches',required=True);p.add_argument('--catch-search',required=True)
    p.add_argument('--pivots',required=True);p.add_argument('--output',required=True);p.add_argument('--record-dir',required=True)
    args=p.parse_args();out=Path(args.output);directory=Path(args.record_dir)
    if out.exists() or directory.exists():p.error('Preserve previous evidence')
    catches=read(args.catches);pivots=read(args.pivots);search=read(args.catch_search)
    if not all(r['completed'] for r in [catches,pivots,search]):p.error('Wait for completed studies')
    if not catches['candidates'] or not pivots['candidates']:p.error('Need qualified catch and pivot candidates')
    caught=max(catches['candidates'],key=lambda c:min(c['result']['peak_roll_deg'],c['coarser_result']['peak_roll_deg']))
    pivot=max(pivots['candidates'],key=lambda c:min(pivot_metrics(c['result'])['score'],pivot_metrics(c['coarser_result'])['score']))
    baseline=search['selected_seeds'][int(caught['id'].split('-')[0])]
    recipes=dict(fall=baseline['recipe'],catch=caught['recipe'],pivot=pivot['recipe'])
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                selected_ids=dict(fall=baseline['id'],catch=caught['id'],pivot=pivot['id']),frozen_recipes=recipes,
                selection_timestep_s=.0005,validation_and_video_timestep_s=.00025,
                nominal={},checks=[],sensitivity=[],sensitivity_seed=2026111954,
                source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('src/reachy_locomotion/motors.py'),Path('configs/motors-speed-trained.json')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    directory.mkdir(parents=True);save()
    config=read('configs/motors-speed-trained.json');config['timestep_s']=.00025
    sim=PrimitiveSimulator('experiments/upstream-reachy-mini',config);recordings={}
    expected=dict(fall=baseline['result'],catch=caught['result'],pivot=pivot['result'])
    for name,recipe in recipes.items():
        r=sim.run_primitive(recipe,seconds=8,waist_mode='hold_neutral',record=True)
        for key in ['termination','peak_roll_deg','peak_tilt_deg','planar_displacement_m','head_braking_work_j','final_foot_yaw_deg']:
            assert r[key]==expected[name][key],(name,key)
        assert r['commanded_waist_position_range_rad']==[0.,0.]
        record_terminal(sim,r);recordings[name]=r
        path=directory/(name+'.json');path.write_text(json.dumps(r,allow_nan=False)+'\n')
        Path(f'configs/primitive-{name}-experimental.json').write_text(json.dumps(recipe,indent=2)+'\n')
        report['nominal'][name]={k:v for k,v in r.items() if k not in ['trace','trajectory','terminal_state']}
        report.setdefault('recording_sha256',{})[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        save();print(json.dumps(dict(stage='recorded',name=name,**summary(dict(id=name,result=r)))),flush=True)
    trigger=recordings['catch']['catch_trigger_s']
    pairs=list(zip(recordings['fall']['trajectory'],recordings['catch']['trajectory']))
    prefix=[(a,b) for a,b in pairs if a['time_s']<=trigger+1e-9]
    assert prefix and abs(prefix[-1][0]['time_s']-trigger)<1e-8
    assert prefix and all(a['time_s']==b['time_s'] and a['qpos']==b['qpos'] and a['qvel']==b['qvel'] for a,b in prefix)
    report['matched_fall_catch_prefix_frames']=len(prefix);report['prefix_matches_through_trigger_s']=trigger
    report['catch_trigger_state']=next(x for x in recordings['catch']['trace'] if abs(x['time_s']-trigger)<1e-8)
    report['recorded_replays_match_selection']=True;save()
    jobs=[]
    for name,g in recipes.items():
        finer=copy.deepcopy(config);finer['timestep_s']=.000125;jobs.append((name+'-0.125ms',g,'hold_neutral',finer))
    no_yaw=dict(recipes['pivot'],yaw_deg=0);jobs.append(('pivot-no-yaw',no_yaw,'hold_neutral',config))
    slow=dict(recipes['pivot'],yaw_s=.5);jobs.append(('pivot-slow-yaw',slow,'hold_neutral',config))
    jobs.append(('pivot-waist-off',recipes['pivot'],'off',config))
    for name in ['catch','pivot']:
        g=copy.deepcopy(recipes[name])
        for key in ['preload_roll_deg','roll_deg','preload_y_mm','y_mm','catch_roll_deg','catch_y_mm','yaw_deg']:g[key]*=-1
        jobs.append((name+'-mirrored',g,'hold_neutral',config))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(check,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(stage='check',**summary(c))),flush=True)
        jobs=[];rng=np.random.default_rng(report['sensitivity_seed'])
        for i in range(4):
            variant=copy.deepcopy(config);variant['friction_slide_torsion_roll'][0]=float(rng.uniform(.55,.9))
            variant['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),
                                         delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
            for name in ['catch','pivot']:jobs.append((f'{name}-model-{i}',recipes[name],'hold_neutral',variant))
        for c in pool.map(check,jobs):
            report['sensitivity'].append(c);save();print(json.dumps(dict(stage='sensitivity',**summary(c))),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
