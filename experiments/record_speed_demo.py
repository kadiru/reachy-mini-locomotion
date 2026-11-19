"""Replay the frozen winner, diagnose motion and compare command-profile ablations."""
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator
from select_speed import stress_job
from train_speed import PROFILES, metrics


def main():
    output=Path('experiments/reports/motor-training-speed-demo-2026-11-19.json')
    recording=Path('experiments/generated/speed-first-20s-2026-11-19.json')
    if output.exists() or recording.exists():raise FileExistsError('Preserve prior evidence')
    selection=json.loads(Path('experiments/reports/motor-training-speed-selection-2026-11-19.json').read_text())
    assert selection['completed']
    recipe=selection['frozen_recipe'];g=Gait(**recipe['gait']);period=round(50/g.frequency_hz)/50
    config=json.loads(Path('configs/motors-speed-trained.json').read_text());config['timestep_s']=.001
    sim=SwaySimulator('experiments/upstream-reachy-mini',config)
    result=sim.run(g,cycles=math.ceil(20/period-1e-10),warmup_cycles=3,record=True,
                   feedback=recipe['feedback'],sensor=json.loads(Path('configs/heading-sensor-assumed.json').read_text()))
    expected=next(x['evaluation']['result'] for x in selection['development'] if x['recipe']['id']==recipe['id'] and x['dt']==.001)
    for key in ['forward_m','lateral_m','max_heading_from_startup_rad','max_foot_tilt_deg','negative_motor_work_j']:
        assert result[key]==expected[key],(key,result[key],expected[key])
    result['render_forward_frame']='startup';recording.write_text(json.dumps(result,allow_nan=False)+'\n')
    rows=[r for r in result['trajectory'] if r['measuring']]
    rpy=np.unwrap(np.array([r['head_rpy_rad'] for r in rows]),axis=0)
    rates=np.gradient(rpy,.02,axis=0);forces=np.array([r['foot_side_normal_forces_n_right_left'] for r in rows])
    slowing=(np.linalg.norm(rates[:,[0,2]],axis=1)>.3)&(np.gradient(np.linalg.norm(rates[:,[0,2]],axis=1),.02)<-1.)
    edge=((forces[:,0]<.1)&(forces[:,1]>.1))|((forces[:,1]<.1)&(forces[:,0]>.1))
    x=np.array([r['foot_xyz_m'] for r in rows]);yaw=result['trajectory'][0]['foot_rpy_rad'][2]
    delta=x-x[0]
    forward=delta[:,0]*math.cos(yaw)+delta[:,1]*math.sin(yaw)
    assert np.isfinite(forward).all()
    assert abs(forward[-1]-result['measurement_displacement_in_startup_frame_m'][0])<1e-12
    stride=round(50/g.frequency_hz);steps=np.diff(forward[::stride])*1000
    power=np.array([r['motor_torque_nm'] for r in rows])*np.array([r['motor_velocity_rad_s'] for r in rows])
    diagnostics=dict(mean_cycle_progress_mm=float(steps.mean()),min_cycle_progress_mm=float(steps.min()),max_cycle_progress_mm=float(steps.max()),
                     single_edge_fraction_50hz=float(edge.mean()),head_slowing_on_edge_fraction=float(edge[slowing].mean()) if slowing.any() else None,
                     head_roll_range_deg=np.degrees([rpy[:,0].min(),rpy[:,0].max()]).tolist(),head_yaw_range_deg=np.degrees([rpy[:,2].min(),rpy[:,2].max()]).tolist(),
                     peak_head_roll_yaw_rate_deg_s=np.degrees(np.max(np.abs(rates[:,[0,2]]),axis=0)).tolist(),
                     peak_observed_forward_speed_mm_s=float(1000*np.gradient(forward,.02).max()),
                     mean_negative_motor_power_w_50hz=float(np.maximum(-power,0).sum(axis=1).mean()),
                     interpretation='kinematic phase correlation, not proof that braking alone causes motion; no-flight classification uses physics-substep clearance and force')
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,recipe=recipe,
                recorded_replay_matches_selection_exactly=True,recording=str(recording),recording_sha256=hashlib.sha256(recording.read_bytes()).hexdigest(),
                nominal={k:v for k,v in result.items() if k!='trajectory'},metrics=metrics(result),diagnostics=diagnostics,ablations=[],completed=False)
    def save():output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    jobs=[('selected',recipe,config,10.)]
    for label,key,value in [('command_acceleration_60','target_acceleration_rad_s2',60.),('command_filter_5','target_filter_hz',5.),('command_speed_6','target_speed_rad_s',6.)]:
        c=copy.deepcopy(config);c['motor_model'][key]=value;jobs.append((label,recipe,c,10.))
    c=copy.deepcopy(config);c['motor_model'].update(PROFILES['existing']);jobs.append(('all_previous_command_profile',recipe,c,10.))
    open_recipe=copy.deepcopy(recipe);open_recipe['feedback']=None;jobs.append(('open_loop',open_recipe,config,10.))
    for name in ['yaw','roll']:
        r=copy.deepcopy(open_recipe);r['gait'][name+'_deg']=0.;r['gait'][name+'_bias_deg']=0.
        jobs.append(('no_'+name,r,config,10.))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(stress_job,jobs):
            report['ablations'].append(r);save()
            print(json.dumps(dict(case=r['case'],**r['metrics'])),flush=True)
    report['completed']=True;save();print(json.dumps(dict(nominal=report['metrics'],diagnostics=diagnostics)),flush=True)


if __name__=='__main__':main()
