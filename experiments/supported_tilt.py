"""Capture supported static poses via GET and compare gravity vectors to the upright recording."""
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit
from urllib.error import HTTPError

import numpy as np

from reachy_locomotion.capture import poll_state
from reachy_locomotion.geometry import angle, euler, transform
from reachy_locomotion.recording import Recorder, load
from reachy_locomotion.robot_io import request_json


def vector(values, size):
    try:
        arr = np.asarray(values, dtype=float).reshape(size)
    except (ValueError, TypeError):
        return None
    return arr if np.isfinite(arr).all() else None


def summarize(rows):
    samples = [row for row in rows if row['kind'] == 'state']
    gravity, rates, tracking, displacement, rtts, roll_errors = [], [], [], [], [], []
    target_rolls, control_modes = [], set()
    for row in samples:
        data = row['data']
        state = data['sdk_state']
        if state.get('control_mode'):
            control_modes.add(state['control_mode'])
        imu = state.get('imu') or {}
        acceleration = vector(imu.get('accelerometer'), 3)
        if acceleration is not None and np.linalg.norm(acceleration) > 1e-6:
            gravity.append(acceleration)
        gyro = vector(imu.get('gyroscope'), 3)
        if gyro is not None:
            rates.append(float(np.degrees(np.linalg.norm(gyro))))
        poses = [vector((state.get(key) or {}).get('m'), 16) for key in ('head_pose','target_head_pose')]
        if all(p is not None for p in poses):
            measured, target = [transform(p.reshape(4, 4)) for p in poses]
            target_rolls.append(float(np.degrees(euler(target)[0])))
            tracking.append(float(np.degrees(angle(measured, target))))
            roll_errors.append(float(np.degrees(euler(target[:3,:3].T@measured[:3,:3])[0])))
            displacement.append(float(1000*(measured[1, 3]-target[1, 3])))
        if data.get('request_t_ns') is not None:
            rtts.append((row['t_ns']-data['request_t_ns'])*1e-6)
    duration = (samples[-1]['t_ns']-samples[0]['t_ns'])*1e-9 if len(samples)>1 else 0.
    med = np.median(gravity, axis=0) if gravity else None
    def p95(values):
        return float(np.percentile(values,95)) if len(values) else None
    return dict(condition=rows[0]['setup']['condition_id'], evidence=rows[0]['evidence'],
                samples=len(samples),imu_samples=len(gravity),motor_modes=sorted(control_modes),
                target_roll_range_deg=[min(target_rolls),max(target_rolls)] if target_rolls else None,read_errors=sum(
                    row['kind']=='event' and row['data']['name']=='state_read_error' for row in rows[1:]),
                receipt_hz=(len(samples)-1)/duration if duration else None,
                request_receipt_p95_ms=p95(rtts),gyro_norm_p95_deg_s=p95(rates),
                gravity_median_m_s2=None if med is None else med.tolist(),
                accel_norm_median_m_s2=float(np.median(np.linalg.norm(gravity,axis=1))) if gravity else None,
                target_present_angle_p95_deg=p95(tracking),target_present_lateral_abs_p95_mm=p95(np.abs(displacement)),
                measured_minus_target_roll_median_deg=float(np.median(roll_errors)) if roll_errors else None,
                measured_minus_target_y_median_mm=float(np.median(displacement)) if displacement else None)


def compare(files):
    rows=[summarize(load(path)) for path in files]
    if rows[0]['condition']!='upright':
        raise ValueError('First recording must be the upright reference')
    if len({row['evidence'] for row in rows})!=1:
        raise ValueError('Use one evidence class per comparison')
    ref=rows[0]['gravity_median_m_s2']
    for row in rows:
        g=row['gravity_median_m_s2']
        row['gravity_change_from_upright_deg']=None
        if ref is not None and g is not None:
            denom=np.linalg.norm(ref)*np.linalg.norm(g)
            if denom>1e-9:
                row['gravity_change_from_upright_deg']=float(np.degrees(np.arccos(np.clip(np.dot(ref,g)/denom,-1.,1.))))
    return rows


def daemon_info(origin):
    url=urlsplit(origin)
    if url.scheme not in ('http','https') or not url.hostname or url.username or url.password or url.path not in ('','/') or url.query or url.fragment:
        raise ValueError('Enter just the Reachy daemon origin')
    try:
        state=request_json(origin.rstrip('/'), '/api/daemon/status', timeout=2.)
    except HTTPError as exc:
        if exc.code!=404:
            raise
        return dict(variant='wireless', source='operator identification; daemon status endpoint unavailable')
    if state.get('simulation_enabled') or state.get('mockup_sim_enabled'):
        raise ValueError('This daemon reports a simulated backend. Physical capture requires the real robot.')
    return {k:state.get(k) for k in ('robot_name','wireless_version','state','simulation_enabled','mockup_sim_enabled')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    subs=parser.add_subparsers(dest='action',required=True)
    cap=subs.add_parser('capture')
    cap.add_argument('condition',choices=('upright','left','right','supported-head'))
    cap.add_argument('--url',required=True)
    cap.add_argument('--output',type=Path,required=True)
    cap.add_argument('--seconds',type=float,default=5.)
    cap.add_argument('--surface',required=True)
    cap.add_argument('--power',choices=('battery','cable','unknown'),default='battery')
    analyze=subs.add_parser('compare')
    analyze.add_argument('files',nargs='+',type=Path)
    args=parser.parse_args()
    if args.action=='compare':
        result=compare(args.files)
    else:
        meta=dict(session_id=args.output.stem,evidence='physical',control_mode='teleoperation',
                  robot=daemon_info(args.url),measurement=dict(time_basis='host receipt'),
                  setup=dict(condition_id=args.condition,surface=args.surface,power=args.power,
                             external_assistance='operator supports torso throughout', acquisition='GET only'))
        with Recorder(args.output,meta) as recorder:
            result=poll_state(recorder,args.url,args.seconds,25)
    print(json.dumps(result,indent=2,allow_nan=False))
    if args.action=='capture' and result['state_samples']==0:
        raise SystemExit(1)


if __name__=='__main__':
    main()
