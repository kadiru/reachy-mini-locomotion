"""Compare rolling and airborne hits on the frozen left/right edge balancing policies."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_balance_robustness import evaluate, read

POLICY = 'configs/balance-recovery-experimental.json'
MOTOR_CONFIG = 'configs/motors-speed-trained.json'
EDGES = ('left', 'right')
MODES = {'rolling': None, 'low throw': .12, 'high throw': .20}
SPEEDS = (.6, 1.2, 2.4)


def grid(direct_only=False):
    if direct_only:
        return [dict(edge=edge, mode='direct', source=source, speed=speed,
                     mass_kg=mass, repeat=False)
                for mass in (.005,.010) for speed in (1.,1.4,2.)
                for edge in EDGES for source in EDGES]
    cases = []
    for repeat in (False, True):
        for edge in EDGES:
            cases.append(dict(edge=edge, mode='control', source=None, speed=0., repeat=repeat))
        for speed in ((1.2,) if repeat else SPEEDS):
            for mode in MODES:
                for edge in EDGES:
                    for source in EDGES:
                        cases.append(dict(edge=edge, mode=mode, source=source, speed=speed, repeat=repeat))
    return cases


def trial_id(case):
    edge, source = case['edge'], case['source'] or 'none'
    batch = 'repeat' if case['repeat'] else 'nominal'
    name = f"{batch}-{case['mode'].replace(' ', '-')}-{case['speed']:g}-{edge}-from-{source}"
    return f"{name}-{1000*case['mass_kg']:g}g" if 'mass_kg' in case else name


def run_case(task):
    case, work = task
    name = trial_id(case)
    path = work / 'states' / (name + '.json')
    bank = read(POLICY)
    policy = bank['edges'][case['edge']]
    release = 6. if case['repeat'] else 5.
    if case['repeat']:
        policy['feedback']['sensor_seed'] = 2026112231
    job = dict(id=name, edge=case['edge'], dt=.000125, seconds=14, policy=policy,
               record=str(path), tilt_limit_deg=None, config=read(MOTOR_CONFIG))
    if case['mode'] != 'control':
        ball = dict(bank['rolling_ball'], from_side=1 if case['source']=='left' else -1,
                    launch_s=release, speed_m_s=case['speed'])
        if case['mode']=='direct':
            # The tilted torso protrudes on the loaded side. Give the sphere a free flight on either side.
            distance = .30 if case['edge']==case['source'] else .21
            ball.update(launch_height_m=.18, distance_m=distance, vertical_speed_m_s=0., mass_kg=case['mass_kg'])
        elif MODES[case['mode']] is not None:
            # Small upward arc. Gravity brings the centre back to launch height after 18 cm horizontally.
            ball.update(launch_height_m=MODES[case['mode']], vertical_speed_m_s=9.81*.18/(2*case['speed']))
        job['balls'] = [ball]
        job['collision_window'] = [release, release+.65]
    evaluate(job)
    run = read(path)
    assert run['natural_upright_start'] and run['commanded_waist_position_range_rad'] == [0.,0.]
    metrics = run['edge_balance']['metrics']
    row = dict(case, id=name, seed=policy['feedback']['sensor_seed'], dt=job['dt'],
               release_s=release if 'balls' in job else None, recorded_end_s=run['elapsed_seconds'],
               termination=run['termination'], final_hold=metrics['final_five_seconds_pass'],
               settled_flat=run['settled'], peak_tilt_deg=run['peak_tilt_deg'])
    if case['mode']=='control':
        row['outcome'] = 'control held' if run['completed'] and row['final_hold'] else 'control failed'
        return row
    event = run['ball_disturbances'][0]
    impact = run['balance_curriculum']['impacts']['events'][0]
    hit, floor = event['first_robot_contact_s'], event['first_floor_contact_s']
    airborne = hit is not None and (floor is None or hit < floor)
    row.update(event=event, balanced_before=impact['balanced_before_impact'], airborne_hit=airborne,
               recovery_s=impact['recovery_s'], peak_roll_change_deg=impact['peak_impact_angle_change_deg'])
    if not event['contacted_robot']:
        outcome = 'miss' if run['completed'] else 'stopped before hit'
    elif case['mode']=='direct' and hit<=event['release_s']+2*job['dt']:
        outcome = 'contact at release'
    elif not impact['balanced_before_impact']:
        outcome = 'unqualified start'
    elif case['mode'] != 'rolling' and not airborne:
        outcome = 'ground first'
    elif run['termination']=='nonfoot_floor_contact':
        outcome = 'toppled'
    elif not run['completed']:
        outcome = 'joint stop' if run['termination']=='actual_joint_limit_exceeded' else run['termination']
    elif run['balance_curriculum']['impact_trial_pass'] and row['final_hold']:
        outcome = 'edge recovery'
    elif run['settled']:
        outcome = 'settled flat'
    else:
        outcome = 'incomplete recovery'
    row['outcome'] = outcome
    return row


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--direct-only', action='store_true', help='Horizontal airborne releases toward both sides')
    args = parser.parse_args()
    if args.work.exists() or args.report.exists():
        parser.error('Choose a new work directory and report')
    args.work.mkdir(parents=True)
    rows = []
    cases = grid(args.direct_only)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for r in pool.map(run_case, ((c, args.work) for c in cases)):
            rows.append(r)
            print(r['id'], r['outcome'], flush=True)
    assert len({r['id'] for r in rows}) == len(cases)
    args.report.write_text(json.dumps(dict(policy=POLICY, motor_model=MOTOR_CONFIG,
        evidence='uncalibrated MuJoCo simulation', rows=rows), indent=2, allow_nan=False)+'\n')
