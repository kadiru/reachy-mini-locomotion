"""Refit a level-ground right-edge catch against the 50 Hz BMI088 simulation."""
import argparse
import hashlib
import json
from tempfile import NamedTemporaryFile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from imu_transfer import FilteredPackets
from imu_rate_fit import RatePackets, coefficients, demonstrate
from imu_reference_catch import LinkedReferenceCatch, ReferenceCatch, TEMPLATE, build
from moving_table import ContactAudit, POLICY, plant, read, stable, support, write
from reachy_locomotion.primitives import Primitive
from train_release_catch import make_controller

WORK = Path('experiments/generated/imu-catch')
PARAMS = Path('configs/imu-right-catch.json')
REPORT = Path('experiments/reports/imu-right-catch.json')
FIT = [(32.2, 0., 4915, 0), (32.2, 0., 27017, 5), (32.2, 0., 37003, 9)]
# Set aside before fitting. Each age sees identical release poses, rates and seeds.
TEST = [(angle, rate, seed, age) for angle, rate, seed in (
    (32.2, 0., 91301), (32.2, 0., 91302), (31.95, .8, 91303), (32.45, -.8, 91304))
    for age in (0, 5, 9)]
TEST += [(angle, rate, seed, age) for angle, rate, seed in (
    (32.2, 0., 91305), (32.05, .4, 91306)) for age in (2.5, 7.5)]


def controller(sim, seed, change):
    if change and change.get('reference'):
        if change.get('linked') and not change.get('replay'):
            return LinkedReferenceCatch(sim, seed, change)
        return ReferenceCatch(sim, seed, change)
    original = read(POLICY)['sides']['1']
    if change and 'weights' in change:
        change = change['regulator']
    if change is None:
        return make_controller(sim, 1, seed, original)
    # Start from the previously trained 32.2° specialist. All releases use the same law.
    params = original['specialists'][original['centers_deg'].index(32.2)]
    params['gain'] *= change['gain']
    params['rate'] *= change['slew']
    params['model']['K'] = np.array(params['model']['K']).copy()
    params['model']['K'][:, 3] *= change['damping']
    params['filter']['attitude_s'] = change['attitude_s']
    params['filter']['command_s'] = change['command_s']
    return make_controller(sim, 1, seed, params)


def run(job, record=False, audit_contact=False):
    change, condition, seconds = job
    angle, rate, seed, age = condition
    sim = plant(.000125, True)
    sensor = RatePackets(age, change['weights']) if change and 'weights' in change else FilteredPackets(age)
    audit = ContactAudit(sim, 1, seconds - 2.) if audit_contact else None
    regulator = controller(sim, seed, change)
    result = sim.run_primitive(Primitive(), seconds=seconds, controller=regulator,
                               waist_mode='hold_neutral', diagnostic_initial_roll_deg=angle,
                               diagnostic_initial_angular_rate_deg_s=rate, record=record,
                               tilt_limit_deg=None, observe_landing=True, record_rim_contact=True,
                               imu_observer=sensor, step_audit=audit)
    trace = result['trace']
    loaded = np.array([support(row, 1) for row in trace])
    roll = np.array([row['roll_deg'] for row in trace])
    tail = [row for row in trace if row['time_s'] >= seconds - 1.]
    safe = (result['completed'] and result['first_body_floor_contact'] is None
            and result['min_joint_margin_rad'] >= 0.)
    # Require two seconds of continuous observed support and a steady final second.
    held = bool(safe and len(loaded) >= 101 and loaded[-101:].all() and stable(tail, 1))
    score = float(np.mean(loaded * np.exp(-abs(roll - 32.1) / 3.)))
    summary = dict(condition=condition, seconds=seconds, held=held, support_fraction=float(loaded.mean()),
                   score=score, shell=result['first_body_floor_contact'], flat=result['settled'],
                   final_roll=float(roll[-1]), margin=result['min_joint_margin_rad'],
                   termination=result['termination'], waist=result['commanded_waist_position_range_rad'])
    if audit_contact:
        summary['max_final_support_gap_ms'] = audit.longest
        summary['held'] = held and audit.loss is None
    if isinstance(regulator, ReferenceCatch):
        summary['corrected_frames'] = regulator.corrections
    if record:
        return summary, result['trajectory'], sim.config, sensor.trace, trace
    return summary


def rank(rows):
    return (sum(r['held'] for r in rows), min(r['score'] for r in rows),
            sum(r['score'] for r in rows), -sum(bool(r['shell']) for r in rows))


def candidates():
    rng = np.random.default_rng(68142)
    for gain in (.45, .70, .95, 1.15):
        for damping in (.6, 1., 1.5):
            yield dict(gain=gain, damping=damping, slew=1., attitude_s=.045, command_s=.009)
    for _ in range(24):
        yield dict(gain=float(rng.uniform(.25, 1.45)), damping=float(rng.uniform(.4, 2.3)),
                   slew=float(rng.uniform(.35, 1.8)), attitude_s=float(rng.uniform(.02, .25)),
                   command_s=float(rng.uniform(.003, .025)))


def replay_path(name, condition):
    angle, rate, seed, age = condition
    return WORK / f'{name}-{angle}-{rate}-{seed}-{age}.npz'


def record_test(job):
    params, condition, _ = job
    summary, trajectory, _, imu, trace = run(job, audit_contact=True, record=True)
    name = policy_name(params)
    path = replay_path(name, condition)
    with NamedTemporaryFile(dir=WORK, delete=False) as temp:
        partial = Path(temp.name)
        try:
            np.savez_compressed(temp, result=json.dumps(summary), params=json.dumps(params),
                                qpos=np.array([p['qpos'] for p in trajectory]),
                                roll=np.array([r['roll_deg'] for r in trace]), estimated_roll=np.array(imu)[:, 1],
                                loaded=np.array([support(r, 1) for r in trace]))
            temp.flush()
            partial.replace(path)
        finally:
            partial.unlink(missing_ok=True)
    return summary


def screen(pool, policies, conditions, seconds, audit_contact=False):
    jobs = [(p, c, seconds) for p in policies for c in conditions]
    results = list(pool.map(record_test if audit_contact else run, jobs))
    n = len(conditions)
    groups = [dict(params=p, trials=results[i*n:(i+1)*n]) for i, p in enumerate(policies)]
    return sorted(groups, key=lambda g: rank(g['trials']), reverse=True)


def fit(pool, workers):
    WORK.mkdir(parents=True, exist_ok=True)
    first = screen(pool, list(candidates()), [FIT[1]], 2.6)
    write(WORK / 'screen.json', first)
    print('2.6 s screen', rank(first[0]['trials']), flush=True)
    finalists = screen(pool, [g['params'] for g in first[:8]], FIT, 4.)
    write(WORK / 'finalists.json', finalists)
    print('4 s finalists', rank(finalists[0]['trials']), flush=True)
    final = screen(pool, [g['params'] for g in finalists[:3]], FIT, 8.)
    write(WORK / 'selection.json', final)
    write(PARAMS, dict(parameters=final[0]['params'], base_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest(),
                       side='right', training_conditions=FIT, workers=workers))
    print('8 s selection', rank(final[0]['trials']), flush=True)


def qualify(pool):
    WORK.mkdir(parents=True, exist_ok=True)
    frozen = read(PARAMS)
    if frozen['base_sha256'] != hashlib.sha256(POLICY.read_bytes()).hexdigest():
        raise ValueError('Base policy has changed since fitting')
    policies = [None, frozen['parameters']]
    if frozen['parameters'].get('reference'):
        if frozen['reference_sha256'] != hashlib.sha256(TEMPLATE.read_bytes()).hexdigest():
            raise ValueError('Reference trajectory changed since fitting')
        policies.append(dict(frozen['parameters'], kp=0., kd=0., shift=0., replay=True))
    groups = screen(pool, policies, TEST, 8., audit_contact=True)
    for group in groups:
        group['name'] = policy_name(group['params'])
        print(group['name'], rank(group['trials']), flush=True)
    digest = hashlib.sha256(PARAMS.read_bytes()).hexdigest()
    old = read(REPORT) if REPORT.exists() else {}
    evidence = old.get('training', {}) if old.get('policy_sha256') == digest else {}
    if (WORK / 'linked-selection.json').exists():
        selection = read(WORK / 'linked-selection.json')
        if selection[0]['params'] == frozen['parameters']:
            stages = ('screen', 'finalists', 'selection', 'observer-screen', 'observer-selection',
                      'reference-screen', 'reference-selection', 'linked-screen', 'linked-selection')
            evidence = {stage: read(WORK / (stage + '.json')) for stage in stages}
    write(REPORT, dict(policy_sha256=digest, qualification=groups, training=evidence))


def fit_observer(pool):
    demos = [(angle, rate, seed, age) for angle, rate, seed in (
        (32.2, 0., 4915), (32.1, .3, 11251), (32.35, -.4, 11252)) for age in (0, 5, 9)]
    records = list(pool.map(demonstrate, demos))
    x, y = [np.concatenate(arrays) for arrays in zip(*records)]
    tuned = read(WORK / 'selection.json')[0]['params']
    policies = [dict(regulator=regulator, weights=coefficients(x, y, penalty))
                for penalty in (1., 50., 500.) for regulator in (None, tuned)]
    rows = screen(pool, policies, FIT, 4.)
    write(WORK / 'observer-screen.json', rows)
    finalists = screen(pool, [r['params'] for r in rows[:3]], FIT, 8.)
    selected = sorted(finalists + read(WORK / 'selection.json'), key=lambda r:rank(r['trials']), reverse=True)
    artifact = read(PARAMS)
    artifact.update(parameters=selected[0]['params'], teacher_conditions=demos, teacher_samples=len(y))
    write(PARAMS, artifact)
    write(WORK / 'observer-selection.json', selected)
    print('8 s observer selection', rank(selected[0]['trials']), flush=True)


def policy_name(params):
    if params is None:
        return 'original'
    return 'replay' if params.get('replay') else 'refit'


def fit_reference(pool):
    build(pool)
    varied = [(32.03, .5, 17301, 0), (32.37, -.5, 17302, 9)]
    options = [dict(reference=True, kp=kp, kd=kd, deadband=pad, shift=shift)
               for kp, kd, shift in ((3., .3, 0.), (12., .9, .05), (30., 2.5, .02), (55., 4., .02),
                                     (-3., -.3, .3), (6., .8, .35))
               for pad in (.08, .2)]
    first = screen(pool, options, [FIT[1], *varied], 3.)
    write(WORK / 'reference-screen.json', first)
    finalists = screen(pool, [r['params'] for r in first[:3]], FIT + varied, 8.)
    previous = read(WORK / 'observer-selection.json')[0]
    previous = screen(pool, [previous['params']], FIT + varied, 8.)[0]
    winners = sorted(finalists + [previous], key=lambda r:rank(r['trials']), reverse=True)
    write(WORK / 'reference-selection.json', winners)
    artifact = read(PARAMS)
    artifact.update(parameters=winners[0]['params'], reference_sha256=hashlib.sha256(TEMPLATE.read_bytes()).hexdigest())
    write(PARAMS, artifact)
    print('8 s reference selection', rank(winners[0]['trials']), flush=True)


def fit_linked(pool):
    build(pool)
    varied = [(32.03, .5, 17301, 0), (32.37, -.5, 17302, 9)]
    candidates = [dict(reference=True, linked=True, gain=g, rate_weight=r, slew=s, deadband=.2)
                  for g in (.45, .85, 1.2) for r in (.65, 1.3) for s in (150., 400.)]
    first = screen(pool, candidates, varied, 3.)
    write(WORK / 'linked-screen.json', first)
    contenders = screen(pool, [r['params'] for r in first[:4]], FIT + varied, 8.)
    ranked = sorted(contenders + read(WORK / 'reference-selection.json'), key=lambda g:rank(g['trials']), reverse=True)
    write(WORK / 'linked-selection.json', ranked)
    artifact = read(PARAMS)
    artifact.update(parameters=ranked[0]['params'], reference_sha256=hashlib.sha256(TEMPLATE.read_bytes()).hexdigest())
    write(PARAMS, artifact)
    print('8 s linked selection', rank(ranked[0]['trials']), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['fit', 'observer', 'reference', 'linked', 'qualify'])
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    with ProcessPoolExecutor(args.workers) as pool:
        if args.phase == 'fit':
            fit(pool, args.workers)
        elif args.phase == 'observer':
            fit_observer(pool)
        elif args.phase == 'reference':
            fit_reference(pool)
        elif args.phase == 'linked':
            fit_linked(pool)
        else:
            qualify(pool)
