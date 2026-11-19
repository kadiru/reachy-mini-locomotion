"""Speed-first head strokes: no smoothness/lateral reward, <=20 deg heading.

Only command waveform/profile choices are searched. Physical torque, mass,
friction and motor speed envelopes stay fixed. No hardware access occurs.
"""
import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator


KEYS = ['frequency_hz', 'roll_deg', 'yaw_deg', 'yaw_phase_deg', 'pitch_bias_deg',
        'x_mm', 'y_mm', 'y_phase_deg', 'z_mm', 'yaw_sharpness', 'roll_sharpness',
        'yaw_bias_deg', 'roll_bias_deg', 'phase_skew', 'pitch_deg', 'pitch_phase_deg',
        'yaw_stroke_fraction', 'roll_stroke_fraction', 'stroke_accel_fraction', 'stroke_brake_fraction']
LOW = np.array([1., 18, 10, -180, -12, -24, -24, -180, -10, 0, 0, -8, -3, -.5, -14, -180, 0, 0, .025, .025])
HIGH = np.array([5., 40, 60, 180, 18, 16, 24, 180, 10, 5, 4, 8, 3, .5, 14, 180, 1, 1, .45, .45])
PROFILES = {
    'existing': dict(target_speed_rad_s=6., target_acceleration_rad_s2=60., target_filter_hz=5.),
    'fast_targets': dict(target_speed_rad_s=9., target_acceleration_rad_s2=180., target_filter_hz=15.),
    'sharp_targets': dict(target_speed_rad_s=10.7, target_acceleration_rad_s2=360., target_filter_hz=25.),
}
CACHE = {}


def metrics(r):
    valid = bool(r['valid']) and r.get('min_motor_joint_limit_margin_rad', 0.) is not None
    valid = valid and r.get('min_motor_joint_limit_margin_rad', 0.) >= -.005
    if not valid or r['measured_seconds'] <= 0:
        return dict(feasible=False, stable=False, speed_mm_s=0., heading_deg=0., score=-1e6)
    forward = r.get('measurement_displacement_in_startup_frame_m', [r['forward_m']])[0]
    speed = 1000 * forward / r['measured_seconds']
    heading = math.degrees(r.get('max_heading_from_startup_rad', r['max_heading_excursion_rad']))
    feasible = heading <= 20 and speed > 0
    score = 1000 + speed if feasible else speed - 3 * max(0., heading - 20)
    return dict(feasible=feasible, stable=True, speed_mm_s=speed, heading_deg=heading, score=score)


def evaluate(payload):
    gait, profile, seconds, dt, feedback = payload
    config = json.loads(Path('configs/motors-datasheet.json').read_text())
    config['timestep_s'] = dt
    config['motor_model'].update(PROFILES[profile])
    key = json.dumps(config, sort_keys=True)
    if key not in CACHE:
        CACHE[key] = SwaySimulator('experiments/upstream-reachy-mini', config)
    g = Gait(**gait)
    period = round(50 / g.frequency_hz) / 50
    result = CACHE[key].run(g, cycles=math.ceil(seconds / period - 1e-10), warmup_cycles=3,
                            feedback=feedback,
                            sensor=json.loads(Path('configs/heading-sensor-assumed.json').read_text()) if feedback else None)
    return dict(gait=gait, profile=profile, metrics=metrics(result), result=result)


def vector(gait):
    g = asdict(Gait(**gait))
    return np.array([g[k] for k in KEYS])


def decode(row):
    g = dict(zip(KEYS, np.clip(row, LOW, HIGH).tolist()))
    g['roll_deg'] = min(g['roll_deg'], 40 - abs(g['roll_bias_deg']))
    g['yaw_deg'] = min(g['yaw_deg'], 60 - abs(g['yaw_bias_deg']))
    for key in ['yaw_stroke_fraction', 'roll_stroke_fraction']:
        if g[key] < .15:
            g[key] = 0.
    return g


def seeds():
    paths = ['configs/sway-progress-fast.json', 'configs/sway-heading-trained.json',
             'configs/sway-candidate.json', 'experiments/reports/motor-training-2026-11-18-policy.json']
    gaits = [json.loads(Path(p).read_text()) for p in paths]
    old = json.loads(Path('experiments/reports/motor-training-progress-2026-11-19.json').read_text())
    candidates = []
    for generation in old['history']:
        if generation['stage'] != 'explore':
            continue
        for c in generation['candidates']:
            r = c['results'][0]
            if r['valid'] and r['max_heading_excursion_rad'] <= math.radians(20):
                candidates.append((r['forward_m_per_s'], c['gait']))
    candidates.sort(key=lambda item: item[0], reverse=True)
    seen = set()
    for _, g in candidates:
        key = json.dumps(g, sort_keys=True)
        if key not in seen:
            gaits.append(g); seen.add(key)
        if len(seen) == 8:
            break
    for width in [.45, .7, .95]:
        for roll in [0., .8]:
            gaits.append(dict(gaits[0], yaw_stroke_fraction=width, roll_stroke_fraction=roll,
                              stroke_accel_fraction=.2, stroke_brake_fraction=.04))
    return gaits


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--baseline-only', action='store_true')
    p.add_argument('--generations', type=int, default=8)
    p.add_argument('--population', type=int, default=36)
    p.add_argument('--seconds', type=float, default=6.6)
    p.add_argument('--seed', type=int, default=2026111911)
    p.add_argument('--profile', choices=PROFILES, default='existing')
    p.add_argument('--seed-report', help='Use additional completed candidates from a prior speed search')
    p.add_argument('--start-gait', help='Keep a second search basin around this specified gait')
    args = p.parse_args(); path = Path(args.output)
    if path.exists():
        p.error('Choose a new report')
    if not 0 <= args.generations <= 20 or not 12 <= args.population <= 64 or not 2 <= args.seconds <= 30:
        p.error('Use a bounded CPU search')
    receipt = dict(evidence='uncalibrated_simulation', hardware_ready=False, args=vars(args),
                   objective='maximize fixed-start-axis forward speed; valid/no-fall and <=20 deg heading including startup',
                   smoothness_reward=False, lateral_penalty=False, no_fall_definition='simulation validity: <=35 deg tilt, no nonfoot floor contact, closure/numerical and joint-limit checks',
                   command_profiles=PROFILES, history=[], completed=False,
                   source_sha256={str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in
                                  [Path(__file__), Path('src/reachy_locomotion/sway.py'), Path('src/reachy_locomotion/motors.py')]})
    seed_gaits = seeds(); rng = np.random.default_rng(args.seed)
    path.parent.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(max_workers=4) as pool:
        if args.baseline_only:
            jobs = [(g, profile, args.seconds, dt, None) for g in seed_gaits[:4]
                    for profile in PROFILES for dt in [.002, .001]]
            receipt['baselines'] = []
            for r in pool.map(evaluate, jobs):
                receipt['baselines'].append(r); path.write_text(json.dumps(receipt, allow_nan=False)+'\n')
                print(json.dumps(dict(profile=r['profile'], frequency=r['gait']['frequency_hz'], dt=r['result']['config']['timestep_s'], **r['metrics'])), flush=True)
        else:
            if args.seed_report:
                previous = json.loads(Path(args.seed_report).read_text())
                old = [c for h in previous['history'] for c in h['candidates']]
                old.sort(key=lambda r:r['metrics']['score'], reverse=True)
                seed_gaits = [r['gait'] for r in old[:8]] + seed_gaits
            initial = list(pool.map(evaluate, [(g, args.profile, args.seconds, .002, None) for g in seed_gaits]))
            initial.sort(key=lambda r:r['metrics']['score'], reverse=True)
            receipt['initial'] = initial
            anchor = json.loads(Path(args.start_gait).read_text()) if args.start_gait else None
            best = initial[0]; mean = vector(anchor or best['gait']); std = .08 * (HIGH - LOW)
            # Search around several moving basins, including explicit brake strokes.
            for generation in range(args.generations):
                samples = rng.normal(mean, std, size=(args.population, len(KEYS)))
                samples[0] = vector(best['gait'])
                for i in range(1, min(7, args.population)):
                    samples[i] = vector(initial[(generation*5+i) % len(initial)]['gait'])
                for i in range(args.population//2, args.population-3):
                    centre = anchor if anchor and i % 2 else best['gait']
                    samples[i] = rng.normal(vector(centre), .035*(HIGH-LOW))
                for i in range(args.population-3, args.population):
                    source = dict(seed_gaits[0], yaw_stroke_fraction=float(rng.uniform(.2, 1)),
                                  roll_stroke_fraction=float(rng.choice([0., .5, .8, 1.])),
                                  stroke_brake_fraction=float(rng.uniform(.025, .12)))
                    samples[i] = rng.normal(vector(source), .025*(HIGH-LOW))
                candidates = list(pool.map(evaluate, [(decode(row), args.profile, args.seconds, .002, None) for row in samples]))
                candidates.sort(key=lambda r:r['metrics']['score'], reverse=True)
                if candidates[0]['metrics']['score'] > best['metrics']['score']:
                    best = candidates[0]
                elites = np.array([vector(r['gait']) for r in candidates[:max(4, args.population//6)]])
                mean = .25*mean + .75*elites.mean(axis=0)
                std = np.maximum(.3*std + .7*elites.std(axis=0), .012*(HIGH-LOW))
                receipt['history'].append(dict(generation=generation, candidates=candidates))
                receipt['best_short_development'] = best
                path.write_text(json.dumps(receipt, allow_nan=False)+'\n')
                print(json.dumps(dict(generation=generation, profile=args.profile, **best['metrics'],
                                      yaw_stroke=best['gait'].get('yaw_stroke_fraction', 0),
                                      roll_stroke=best['gait'].get('roll_stroke_fraction', 0))), flush=True)
    receipt['completed'] = True; path.write_text(json.dumps(receipt, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
