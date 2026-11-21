"""Fit bounded active-waist arcs on a continuously loaded left or right rim."""
import argparse
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.waist_balance import WaistBalance
from train_target_pivot import read, write
from audit_waist_rim import RimAudit

WORK = Path('experiments/generated/waist-balance')
REPORT = Path('experiments/reports/motor-training-waist-balance.json')
BASELINE = 'configs/edge-balance-experimental.json'
MOTORS = 'configs/motors-speed-trained.json'
CONDITIONS = {
    'nominal': (.7, 5., 1., 0., .000125),
    'floor': (.67, 5., 1., 0., .000125),
    'battery': (.7, 4.95, .99, 0., .000125),
    'sensing': (.7, 5., 1., .03, .000125),
    'timestep': (.7, 5., 1., 0., .0001),
}
CACHE = {}


def measures(result, ctl, job):
    start = ctl.start
    active = [r for r in result['trace'] if start is not None and r['time_s'] >= start]
    side = ctl.side
    angle_errors = []
    prefix = []
    for row in active:
        point = row['contact_foot_xy_m']
        if point is None:
            break
        angle = abs(math.degrees(math.atan2(point[0], -side * point[1])))
        if not row['edge_support'] or side * row['roll_deg'] <= 0 or angle > 15:
            break
        prefix.append(row)
        angle_errors.append(angle)
    def span(key):
        values = [row[key] for row in prefix]
        return [min(values), max(values)] if values else [0., 0.]
    waist = span('waist_angle_change_deg')
    elapsed = prefix[-1]['time_s'] - start if prefix else 0.
    rotations = int((elapsed + 1e-8) / job['period'])
    enough = job['amplitude'] == 0 or min(-waist[0], waist[1]) >= .85 * job['amplitude']
    turns = [prefix[round(k*job['period']*50)] for k in range(rotations+1)] if prefix else []
    increments = [round(b['torso_yaw_deg']-a['torso_yaw_deg'], 5) for a,b in zip(turns, turns[1:])]
    stable_fraction = sum(abs(side*r['roll_deg']-ctl.f.target_roll_deg)<=6 for r in prefix)/max(1,len(active))
    torso = span('torso_yaw_deg')
    world_turn = torso[1] - torso[0] >= job['amplitude']

    passed = (result['completed'] and result['min_joint_margin_rad'] >= 0 and active
              and len(active) == len(prefix) and rotations >= 1 and enough and world_turn and stable_fraction >= .95)
    return dict(start_s=start, same_edge_s=elapsed, stable_fraction=stable_fraction, cycle_torso_net_deg=increments, full_cycles=rotations, waist_range_deg=waist,
                foot_yaw_range_deg=span('foot_yaw_deg'), torso_yaw_range_deg=span('torso_yaw_deg'),
                roll_range_deg=span('roll_deg'), rim_azimuth_error_deg=max(angle_errors, default=None),
                sampled_same_edge_through_end=bool(active and len(active) == len(prefix)),
                first_loss_s=active[len(prefix)]['time_s'] if len(prefix) < len(active) else None,
                peak_waist_torque_nm=result['waist_peak_torque_nm'],
                actual_joint_margin_rad=result['min_joint_margin_rad'], termination=result['termination'],
                fell=result['fell'], command_rad=result['commanded_waist_position_range_rad'],
                elapsed_s=result['elapsed_seconds'], passed=bool(passed))


def trial(job, record=None):
    condition = job.get('condition', 'nominal')
    friction, voltage, gain, noise, dt = CONDITIONS[condition]
    if condition not in CACHE:
        if len(CACHE) >= 2:
            CACHE.pop(next(iter(CACHE)))
        config = read(MOTORS)
        config['timestep_s'] = dt
        config['friction_slide_torsion_roll'][0] = friction
        config['motor_model'].update(voltage_v=voltage, pid_scale=gain)
        CACHE[condition] = PrimitiveSimulator('experiments/upstream-reachy-mini', config)
    sim = CACHE[condition]
    side = job['side']
    policy = read(BASELINE)['sides']['left' if side < 0 else 'right']
    policy['feedback'].update(sensor_noise_deg=noise, sensor_seed=3819)
    ctl = WaistBalance(policy, side, sim.checked_ik, job['amplitude'], job['period'],
                       direction=job['direction'], compensate=job.get('compensate', True),
                       freeze_head=job.get('freeze', False), com_offset_mm=job.get('com_offset_mm', 0.),
                       gain_change=job.get('gain_change', 0.), return_fraction=job.get('return_fraction', .5))
    audit = RimAudit(ctl) if job.get('audit') else None
    result = sim.run_primitive(Primitive(**policy['recipe']), seconds=job['seconds'], waist_mode='position',
                               record=bool(record), controller=ctl, tilt_limit_deg=None, step_audit=audit)
    m = measures(result, ctl, job)
    if audit:
        m['contact_audit'] = audit.report()
        m['passed'] = m['passed'] and audit.steps > 0 and audit.loaded_steps == audit.steps
    if record:
        target_angles = np.degrees([frame['target_head_rpy_rad'] for frame in result['trajectory']])
        waist_angles = np.array([r['waist_target_deg'] for r in result['trace']])
        delta = (target_angles[:,2] - waist_angles + 180) % 360 - 180
        diagnostics = dict(peak_requested_head_roll_pitch_deg=np.abs(target_angles[:,:2]).max(axis=0).tolist(),
                           peak_requested_head_to_body_yaw_deg=float(np.abs(delta).max()),
                           peak_motor_torque_nm=result['peak_motor_torque_nm'],
                           torque_caps_nm=result['motor_diagnostics']['torque_caps_nm'])
        write(record, dict(title='Turning while balanced on one edge', config=result['config'], diagnostics=diagnostics,
                           metrics=dict(mode='waist_balance', side=side, period_s=job['period'], **m), trace=result['trace'],
                           trajectory=[dict(qpos=r['qpos']) for r in result['trajectory']]))
    return dict(job=job, **m)


def job(side, amplitude, period, seconds=None, direction=1, **kwargs):
    return dict(side=side, amplitude=amplitude, period=period, seconds=seconds or period+5,
                direction=direction, **kwargs)


def batch(jobs, workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(trial, jobs))


def show(rows):
    for r in rows:
        j = r['job']
        print(j['side'], j['amplitude'], j['period'], j['direction'], j.get('condition','nominal'), r['start_s'],
              round(r['same_edge_s'],2), list(np.round(r['waist_range_deg'],1)), r['termination'], r['passed'], flush=True)


def screen(workers):
    jobs = [job(s, a, p) for s in (-1,1) for a,p in ((0,8),(12,8),(35,8),(70,12),(120,16),(155,16))]
    rows = batch(jobs, workers)
    write(WORK/'screen.json', rows)
    show(rows)


def fit(workers):
    # A paired search near the failed large arcs. Lower gains at high yaw can
    # reduce oscillation as the head mechanism turns relative to the edge.
    candidates = [(80,12,0), (95,12,0), (110,14,0), (145,20,-.35), (155,18,-.55),
                  (155,20,-.35), (155,22,.25), (130,16,-.35), (110,14,-.35),
                  (100,12,-.5), (90,12,.25)]
    jobs = [job(s,a,p,gain_change=g) for a,p,g in candidates for s in (-1,1)]
    rows = batch(jobs, workers)
    write(WORK/'fit.json', rows)
    show(rows)


def fit_offsets(workers):
    candidates = [(a,period,offset) for a,period in ((110,14),(135,18),(155,20))
                  for offset in (-6.,3.,9.)]
    jobs = [job(side,a,p,com_offset_mm=offset) for a,p,offset in candidates for side in (-1,1)]
    rows = batch(jobs,workers)
    write(WORK/'offsets.json',rows)
    show(rows)


def baseline_long(workers):
    jobs = [job(s,70,12,41,audit=True) for s in (-1,1)]
    rows = batch(jobs, workers)
    write(WORK/'baseline-long.json', rows)
    show(rows)


POLICY = Path('configs/waist-edge-sweep.json')


def endurance(workers):
    rows = read(WORK/'fit.json') + read(WORK/'offsets.json')
    pairs = [rows[i:i+2] for i in range(0, len(rows), 2)]
    finalists = sorted((pair for pair in pairs if all(r['passed'] for r in pair)),
                       key=lambda pair: pair[0]['job']['amplitude'], reverse=True)[:3]
    jobs = [dict(row['job'], seconds=3*row['job']['period']+5, audit=True)
            for pair in finalists for row in pair]
    finalists = batch(jobs, workers)
    baseline = read(WORK/'baseline-long.json')
    if any(r['contact_audit']['samples'] == 0 for r in baseline):
        raise ValueError('Endurance baseline lacks a physics-step contact audit')
    long_runs = baseline + finalists
    write(WORK/'endurance.json', long_runs)
    pairs = [long_runs[i:i+2] for i in range(0,len(long_runs),2)]
    winner = max(pairs, key=lambda pair: (sum(r['passed'] for r in pair),
                                          min(r['full_cycles'] for r in pair),
                                          pair[0]['job']['amplitude']))
    selected = winner[0]['job']
    write(POLICY, dict(base_policy=BASELINE, motors=MOTORS, amplitude=selected['amplitude'],
                       period=selected['period'], gain_change=selected.get('gain_change',0.),
                       com_offset_mm=selected.get('com_offset_mm',0.), integration_step_s=CONDITIONS['nominal'][-1]))
    show(long_runs)


def sustained(workers):
    p = read(POLICY)
    jobs = [job(side,p['amplitude'],p['period'],seconds=10*p['period']+5,
                gain_change=p['gain_change'],com_offset_mm=p['com_offset_mm'],audit=True) for side in (-1,1)]
    result = batch(jobs,workers)
    write(WORK/'sustained.json',result)
    show(result)


def qualify(workers):
    p = read(POLICY)
    def make(side, **kwargs):
        return job(side,p['amplitude'],p['period'],gain_change=p['gain_change'],com_offset_mm=p['com_offset_mm'],audit=True,**kwargs)
    tests = [make(s,condition=c) for c in ('floor','battery','sensing','timestep') for s in (-1,1)]
    tests += [make(s,direction=-1) for s in (-1,1)]
    controls = [make(s,freeze=True) for s in (-1,1)] + [make(s,compensate=False) for s in (-1,1)]
    # The existing neutral-waist capture is already measured by screen().
    # A fast return stroke tests for accumulated world heading after the waist rewinds.
    ratchet = [job(s,55,10,35,direction=d,return_fraction=.25,audit=True)
               for s in (-1,1) for d in (1,-1)]
    rs = batch(tests+controls+ratchet,workers)
    report = dict(simulation_only=True, policy=p, conditions=CONDITIONS, screen=read(WORK/'screen.json'),
                  fit=read(WORK/'fit.json'), offsets=read(WORK/'offsets.json'), endurance=read(WORK/'endurance.json'),
                  challenges=rs[:8], reverse_start=rs[8:10], controls=rs[10:14], asymmetric=rs[14:])
    write(REPORT,report)
    show(rs)


def record(workers):
    p = read(POLICY)
    report = read(REPORT) if REPORT.exists() else dict(policy=p, endurance=read(WORK/'endurance.json'))
    if p != report['policy']:
        raise ValueError('Policy and qualification differ')
    rows = [row for row in report['endurance'] if row['job']['amplitude'] == p['amplitude']
            and row['job']['period'] == p['period'] and row['job'].get('gain_change',0.) == p['gain_change']
            and row['job'].get('com_offset_mm',0.) == p['com_offset_mm']]
    if sorted(row['job']['side'] for row in rows) != [-1,1]:
        raise ValueError('Expected the two selected endurance runs')
    paths = [WORK / ('left.json' if row['job']['side'] < 0 else 'right.json') for row in rows]
    with ProcessPoolExecutor(max_workers=min(2, workers)) as pool:
        replays = list(pool.map(trial, [row['job'] for row in rows], paths))
    if replays != rows:
        raise ValueError('Replay differs from the endurance tests')


def summarize(workers):
    report = read(REPORT)
    policy = read(POLICY)
    if report['policy'] != policy:
        raise ValueError('Frozen policy and qualification differ')
    long_runs = read(WORK/'sustained.json')
    for r in long_runs:
        task = r['job']
        if (task['amplitude'], task['period'], task['gain_change'], task['com_offset_mm']) != (
                policy['amplitude'], policy['period'], policy['gain_change'], policy['com_offset_mm']):
            raise ValueError('Long-run policy differs from qualification')
    report['sustained'] = long_runs
    report['film_diagnostics'] = {}
    for side in ('left','right'):
        film = read(WORK/f'{side}.json')
        active = [row for row in film['trace'] if row['time_s'] >= film['metrics']['start_s']]
        xy = np.array([row['contact_world_m'][:2] for row in active if row['contact_world_m']])
        diagnostics = film['diagnostics']
        diagnostics['max_floor_contact_excursion_mm'] = float(1000*np.linalg.norm(xy-xy[0],axis=1).max())
        report['film_diagnostics'][side] = diagnostics
    write(REPORT, report)


if __name__ == '__main__':
    stages = (screen,fit,fit_offsets,baseline_long,endurance,sustained,qualify,record,summarize)
    commands = {stage.__name__:stage for stage in stages}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=commands)
    parser.add_argument('--workers',type=int,default=6)
    args=parser.parse_args()
    commands[args.stage](args.workers)
