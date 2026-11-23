"""Matched, frozen-policy IMU rate sweep on the original MuJoCo plants."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from moving_table import read
from multirate_imu import feed_archived_runner

RATES = ('archive', 'stock', 25, 50, 100, 200, 400, 500, 1000)
CORE = ('refit release', 'refit shifted', 'original release', 'left CAD release', 'moving deck',
        'front edge', 'back edge', 'left edge', 'right edge', 'rolling impact', 'direct impact', 'controlled return')
FULL_LENGTH = ('large rocking','wide gyro peg','side shell rocking')
WORK = Path('experiments/generated/frequency-study')


def cfg(name):
    return read('configs/' + name + '.json')


def metric(passed, value, unit, **details):
    return dict(passed=bool(passed), value=float(value), unit=unit, **details)


def core_case(name, repeat):
    seed = (4915, 91304)[repeat]
    if name in ('refit release','refit shifted','original release'):
        from train_imu_catch import run, PARAMS
        change = None if name.startswith('original') else read(PARAMS)['parameters']
        angle, speed = (32.45, -.8) if name == 'refit shifted' else (32.2, 0.)
        row = run((change, (angle, speed, seed, 5 * repeat), 8.), audit_contact=True)
        return metric(row['held'], row['support_fraction'], 'loaded fraction', flat=bool(row['flat']),
                      shell=bool(row['shell']), final_deg=row['final_roll'])
    if name == 'left CAD release':
        from train_release_catch import simulate, job
        row = simulate(job(-1, 32.2, 0., cfg('cad-release-catch')['sides']['-1'], 8., seed))
        return metric(row['strict'],row['balance']['longest_strict_hold_s'],'hold s',outcome=row['outcome'])
    if name == 'moving deck':
        from moving_table import case, simulate
        from qualify_moving_table import edge_pass
        row = simulate(case(1, 10., seed=seed))
        return metric(edge_pass(row),row['peak_table_deg'],'deck deg',flat=bool(row['flat_return']),shell=bool(row['shell']))
    if name in ('front edge', 'back edge'):
        from train_foreaft_balance import evaluate
        edge = name.split()[0]
        side = 1 if edge == 'front' else -1
        policy = cfg('foreaft-balance-experimental')['edges'][edge]
        if repeat:
            policy['feedback']['sensor_seed'] = seed
        row = evaluate(dict(id=name, policy=policy, side=side, diagnostic_initial_pitch_deg=32.1*side, seconds=8.))
        m = row['metrics']
        return metric(m['final_five_seconds_pass'],m['longest_strict_hold_s'],'hold s')
    from train_balance_robustness import evaluate
    bank = cfg('balance-recovery-experimental')
    edge = 'left' if name == 'left edge' else 'right'
    policy = bank['edges'][edge]
    if repeat:
        policy['feedback']['sensor_seed'] = seed
    job = dict(id=name, policy=policy, edge=edge, seconds=10., dt=.000125, tilt_limit_deg=None)
    if name == 'controlled return':
        policy['return'] = bank['controlled_return']
        job['seconds'] = 14.
    if name.endswith('impact'):
        ball = dict(bank['rolling_ball'], from_side=-1, launch_s=5., speed_m_s=.3)
        if name == 'direct impact':
            ball.update(launch_height_m=.18, distance_m=.30, vertical_speed_m_s=0., mass_kg=.005, speed_m_s=1.4)
        job.update(balls=[ball], seconds=14.)
    row = evaluate(job)
    m = row['metrics']
    c = row['result']['balance_curriculum']
    passed = m['final_five_seconds_pass']
    if name == 'controlled return':
        passed = c['hold_return_pass']
    elif name.endswith('impact'):
        passed = c['impact_trial_pass']
    return metric(passed, m['longest_strict_hold_s'],'hold s', termination=row['result']['termination'])


def evaluate(job):
    name, rate, repeat = job
    with feed_archived_runner(rate, repeat * .5) as streams:
        if name in CORE:
            result = core_case(name, repeat)
        else:
            from imu_skill_cases import motion_case
            result = motion_case(name, repeat)
    if rate != 'archive' and not streams:
        raise RuntimeError(f'{name} did not run an instrumented simulation')
    updates = sum(len(s.trace) if rate == 'stock' else s.packets for s in streams)
    return dict(skill=name, rate=rate, repeat=repeat, imu_updates=updates, **result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('group')
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--archive-only', action='store_true')
    args = parser.parse_args()
    if args.group == 'core':
        skills = CORE
    else:
        from imu_skill_cases import GROUPS, HISTORICAL, OPEN_LOOP
        groups = dict(GROUPS, extended=sum(GROUPS.values(), ()), historical=HISTORICAL, openloop=OPEN_LOOP, reference=FULL_LENGTH, heading=('heading gait',))
        skills = groups[args.group]
    WORK.mkdir(exist_ok=True, parents=True)
    output = WORK / (args.group + '.jsonl')
    existing = [json.loads(line) for line in output.read_text().splitlines()] if output.exists() else []
    done = {(r['skill'],r['rate'],r['repeat']) for r in existing}
    rates = ('archive',) if args.archive_only else RATES
    jobs = [(s, hz, rep) for hz in rates for s in skills for rep in range(2) if (s,hz,rep) not in done]
    from concurrent.futures import as_completed
    with ProcessPoolExecutor(args.workers) as pool, output.open('a') as f:
        futures = {pool.submit(evaluate,j):j for j in jobs}
        failures = []
        for future in as_completed(futures):
            try:
                row = future.result()
                f.write(json.dumps(row, allow_nan=False) + '\n')
                f.flush()
            except Exception as exc:
                failures.append(str((futures[future], repr(exc))))
                if len(failures) <= 3:
                    print(failures[-1], flush=True)
        if failures:
            raise RuntimeError(f'{len(failures)} unfinished trials: {failures[-1]}')
    print(output, flush=True)


if __name__ == '__main__':
    main()
