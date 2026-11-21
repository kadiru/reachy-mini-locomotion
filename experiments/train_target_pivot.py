"""Fit and qualify a heading-conditioned edge pivot in the free-base simulator."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.target_pivot import TargetPivot, pivot_metrics


def read(path):
    return json.loads(Path(path).read_text())


def write(path, obj):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


# sliding friction, bus voltage, gain scale, command delay, mass allocation,
# integration timestep, observation delay, sensor noise, sensor RNG seed
CONDITIONS = {
    'fit': (.7, 5., 1., 1, 'torso', .0005, 1, .12, 21),
    'dev': (.64, 4.9, .97, 1, 'torso', .0005, 1, .15, 37),
    'low_fit': (.58, 4.82, .95, 1, 'torso', .0005, 2, .18, 41),
    'high_fit': (.84, 5.28, 1.06, 1, 'torso', .0005, 1, .17, 42),
    'mass_fit': (.74, 5.1, 1., 2, 'uniform', .0005, 1, .15, 43),
    'fresh_low': (.60, 4.86, .96, 1, 'torso', .0005, 2, .18, 213),
    'fresh_high': (.80, 5.23, 1.04, 1, 'torso', .0005, 1, .15, 215),
    'fresh_mass': (.79, 5.03, 1.03, 2, 'uniform', .0005, 1, .18, 216),
    'unseen_low': (.57, 4.84, .943, 1, 'torso', .0004, 2, .19, 391),
    'unseen_inertia': (.77, 5.06, 1.025, 2, 'uniform', .0004, 1, .17, 392),
    'unseen_grip': (.83, 5.27, 1.045, 1, 'torso', .0004, 1, .14, 393),
    'nominal': (.7, 5., 1., 1, 'torso', .0005, 1, .12, 101),
    'slick': (.55, 4.8, .94, 1, 'torso', .0005, 2, .2, 102),
    'grip': (.85, 5.35, 1.07, 1, 'torso', .0005, 1, .17, 103),
    'inertia': (.76, 5.1, 1.02, 2, 'uniform', .0005, 1, .16, 104),
    'fine125': (.7, 5., 1., 1, 'torso', .000125, 1, .12, 218),
    'fine': (.7, 5., 1., 1, 'torso', .00025, 1, .12, 105),
}
CACHE = {}
BANK = 'configs/target-edge-pivot.json'
REPORT = 'experiments/reports/motor-training-target-edge-pivot.json'
FIT = 'experiments/generated/target-pivot-fit.json'
FIT_SUMMARY = 'experiments/reports/motor-training-target-edge-pivot-fit.json'


def archive_fit(rows):
    compact = [dict(option=r['strokes'][0]['option'], condition=r['condition'], settled=r['settled'],
                    termination=r['termination'], torso_deg=r['torso_deg'], edge_deg=r['signed_edge_deg'],
                    lift_mm=r['lift_mm'], drift_mm=r['drift_mm'], slip_mm=r['slip_mm']) for r in rows]
    write(FIT_SUMMARY, dict(evidence='uncalibrated_simulation', results=compact))


def options():
    source = read('configs/penguin-sustained-trained.json')
    for family in ('primary', 'fallback', 'delayed_sensor_fallback'):
        parent = source[family]
        for yaw in (10, 25, 40, 55):
            for side in (-1, 1):
                yield dict(id=f'{family}-{yaw}-{side:+d}', side=side, turn_deg=-side,
                           recipe=dict(parent['recipe'], yaw_deg=yaw), feedback=parent['feedback'])


    # Search for larger turns and for low-translation finishing strokes.
    rng = np.random.default_rng(718)
    parent = source['primary']
    for index in range(32):
        recipe = dict(parent['recipe'], x_mm=float(rng.uniform(-16, 19)),
                      pitch_deg=float(rng.uniform(-12, 12)), pump_hz=float(rng.uniform(2.45,3.2)),
                      yaw_deg=int(rng.choice([-55,-30,30,55])), yaw_hold_s=float(rng.uniform(.1,.45)))
        feedback = dict(parent['feedback'], trigger_deg=int(rng.choice([8,10,13,15])),
                        yaw_delay_after_catch_s=0., yaw_before_catch=bool(index%2))
        for side in (-1, 1):
            yield dict(id=f'search-{index}-{side:+d}',side=side,turn_deg=-side,recipe=recipe,feedback=feedback)


def run(job):
    bank, target, condition, single, replay, *override = job
    friction, volts, gain, delay, mass, dt, obs_delay, noise, seed = CONDITIONS[condition]
    if condition not in CACHE:
        cfg = read('configs/motors-speed-trained.json')
        cfg['timestep_s'] = dt
        cfg['friction_slide_torsion_roll'][0] = friction
        cfg['motor_model'].update(voltage_v=volts, pid_scale=gain, delay_frames=delay, mass_distribution=mass)
        CACHE[condition] = PrimitiveSimulator('experiments/upstream-reachy-mini', cfg)
    sim = CACHE[condition]
    settings = override[0] if override else ({} if single else read(BANK).get('control', {}))
    policy = TargetPivot(bank, target, sim.checked_ik, single=single, seed=seed, delay=obs_delay, **settings,
                         noise=noise, max_strokes=1 if single else 9)
    result = sim.run_primitive(Primitive(), seconds=5.6 if single else 41., waist_mode='hold_neutral',
                               controller=policy, record=bool(replay), tilt_limit_deg=None, finish_settle_s=.8)
    metrics = pivot_metrics(result, target)
    metrics.update(condition=condition, strokes=policy.strokes, peak_tilt_deg=result['peak_tilt_deg'],
                   min_joint_margin_rad=result['min_joint_margin_rad'], max_closure_error_m=result['max_closure_error_m'],
                   peak_motor_torque_nm=max(result['peak_motor_torque_nm']), natural_upright_start=result['natural_upright_start'])
    metrics['mean_edge_slip_mm_per_stroke'] = metrics['slip_mm'] / max(1, len(policy.strokes))
    metrics['passed'] = metrics['passed'] and metrics['mean_edge_slip_mm_per_stroke'] <= 15.
    if replay:
        trajectory = [{k: row[k] for k in ('time_s','qpos','foot_rpy_rad')} for row in result['trajectory']]
        trace = [{k: row[k] for k in ('time_s','foot_xyz_m','foot_yaw_deg','torso_yaw_deg','roll_deg',
                                       'edge_support','side_height_difference_mm')} for row in result['trace']]
        write(replay, dict(config=result['config'], trajectory=trajectory, trace=trace, metrics=metrics,
                           terminal_state=result['terminal_state'], elapsed_seconds=result['elapsed_seconds']))
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('fit', 'refine', 'tune', 'qualify', 'replay'))
    parser.add_argument('--target', type=float, default=30.)
    parser.add_argument('--condition', choices=CONDITIONS, default='nominal')
    parser.add_argument('--output')
    parser.add_argument('--record-dir', type=Path, help='Save all qualification trajectories here')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if args.mode == 'replay':
        if not args.output:
            parser.error('Supply a trajectory --output for replay')
        print(json.dumps(run((read(BANK)['options'], args.target, args.condition, False, args.output))))
        return
    if args.mode == 'fit':
        candidates = list(options())
        fit_conditions = ('fit', 'dev', 'low_fit', 'high_fit', 'mass_fit')
        for opt in candidates:
            # Reserve command-space joint travel for servo overshoot at contact.
            opt['feedback'] = dict(opt['feedback'], workspace_joint_margin_rad=.065)
        jobs = [([opt], 0., cond, True, None) for opt in candidates for cond in fit_conditions]
    elif args.mode == 'refine':
        policy = read(BANK)
        candidates = policy['options']
        # Joint events during multi-stroke development exclude the triggering
        # motion even when it passed all five single-stroke training starts.
        development = read('experiments/reports/motor-training-target-edge-development.json')['results']
        unsafe = {r['strokes'][-1]['option'] for r in development if r['termination'] == 'actual_joint_limit_exceeded'}
        candidates = [o for o in candidates if o['id'] not in unsafe]
        extra = ('slick', 'fresh_low')
        if any(c in policy['fit_conditions'] for c in extra):
            parser.error('Run fit before repeating refinement')
        jobs = [([opt], 0., cond, True, None) for opt in candidates for cond in extra]
    elif args.mode == 'tune':
        bank = read(BANK)['options']
        controls = [dict(delayed_edge_weight=w, repeat_margin=m) for w,m in
                    ((3.,.065), (.2,.1), (.6,.13), (1.2,.15))]
        cases = [(g,c) for c in ('slick','fresh_low','inertia','fresh_mass') for g in (-30,30)]
        jobs = [(bank,g,c,False,None,control) for control in controls for g,c in cases]
    else:
        bank = read(BANK)['options']
        jobs = [(bank, goal, cond, False, None) for cond in
                ('nominal', 'slick', 'grip', 'inertia', 'fine', 'fresh_low', 'fresh_high','fresh_mass')
                for goal in (-45, -30, -17, 17, 30, 45)]
        jobs.extend((bank, goal, 'fine125', False, None) for goal in (-30,30))
        jobs.extend((bank, goal, cond, False, None) for cond in ('unseen_low','unseen_inertia','unseen_grip')
                    for goal in (-42,-24,24,42))
        jobs.append((bank, 0., 'nominal', False, None))
        jobs.extend((bank, goal, 'nominal', False, None) for goal in (-90,-60,60,90))
    if args.record_dir and args.mode == 'qualify':
        jobs = [(b, g, c, single, str(args.record_dir/f'{c}-{g:g}.json')) for b,g,c,single,_ in jobs]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run, jobs))
    if args.mode == 'fit':
        by_condition = len(fit_conditions)
        bank = []
        for i, opt in enumerate(candidates):
            sample = rows[i*by_condition:(i+1)*by_condition]
            if any(r['termination'] != 'completed' or not r['settled'] for r in sample):
                continue
            opt['responses'] = [dict(turn=r['torso_deg'], edge=r['signed_edge_deg'], drift=r['drift_mm'],
                                     roll=r['strokes'][0]['peak_sensed_roll_deg'],
                                     delay=CONDITIONS[r['condition']][6]) for r in sample]
            opt['turn_deg'] = sample[0]['torso_deg']
            bank.append(opt)
        write(FIT, rows)
        archive_fit(rows)
        write(BANK, dict(options=bank, fit_conditions=fit_conditions))
        print(f'{len(bank)} motions survived all {len(fit_conditions)} development conditions')
    elif args.mode == 'refine':
        bank = []
        for i, opt in enumerate(candidates):
            sample = rows[i*len(extra):(i+1)*len(extra)]
            if not all(r['settled'] and r['termination']=='completed' for r in sample):
                continue
            opt['responses'].extend(dict(turn=r['torso_deg'],edge=r['signed_edge_deg'],drift=r['drift_mm'],
                                         roll=r['strokes'][0]['peak_sensed_roll_deg'],delay=CONDITIONS[r['condition']][6])
                                    for r in sample)
            bank.append(opt)
        write(BANK, dict(options=bank, fit_conditions=list(policy['fit_conditions']) + list(extra)))
        write('experiments/reports/motor-training-target-edge-refinement.json',
              dict(excluded_after_joint_stop=sorted(unsafe), results=rows))
        print(f'Retained {len(bank)} motions after refinement')
    elif args.mode == 'tune':
        trials = []
        for i, control in enumerate(controls):
            subset = rows[i*len(cases):(i+1)*len(cases)]
            score = (sum(r['passed'] for r in subset), sum(r['settled'] for r in subset),
                     sum(r['max_tail_error_deg'] <= 5 for r in subset), -sum(r['max_tail_error_deg'] for r in subset))
            trials.append(dict(settings=control,score=score,results=subset))
        winner = max(trials,key=lambda x:x['score'])
        policy = read(BANK)
        policy['control'] = winner['settings']
        write(BANK,policy)
        write('experiments/reports/motor-training-target-edge-feedback.json',dict(trials=trials))
        print([(t['settings'],t['score']) for t in trials])
    else:
        write(args.output or REPORT, dict(evidence='uncalibrated_simulation', hardware_ready=False,
                                          policy_sha256=hashlib.sha256(Path(BANK).read_bytes()).hexdigest(),
                                          waist_mode='hold_neutral', tilt_limit_deg=None,
                                          condition_fields=['sliding_friction','voltage_v','pid_scale','command_delay_frames',
                                                            'mass_distribution','timestep_s','sensor_delay_frames',
                                                            'sensor_noise_deg','sensor_seed'],
                                          conditions=CONDITIONS, results=rows))
        for r in rows:
            print(r['condition'],r['target_deg'],round(r['torso_deg'],1),round(r['signed_edge_deg'],1),
                  round(r['max_excursion_mm'],1),r['passed'],r['termination'])


if __name__ == '__main__':
    main()
