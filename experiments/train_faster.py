"""Larger swings and sharper braking with the speed-first motor config frozen.

CPU-only parameter optimization. All motion enters through existing IK/servos;
no changes to mass, contact, torque, command limits or the physical motor model.
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
from train_speed import metrics


KEYS = ['frequency_hz', 'roll_deg', 'yaw_deg', 'yaw_phase_deg', 'pitch_bias_deg',
        'x_mm', 'y_mm', 'y_phase_deg', 'z_mm', 'yaw_sharpness', 'roll_sharpness',
        'yaw_bias_deg', 'roll_bias_deg', 'phase_skew', 'pitch_deg', 'pitch_phase_deg',
        'yaw_stroke_fraction', 'roll_stroke_fraction', 'stroke_accel_fraction',
        'stroke_brake_fraction', 'yaw_harmonic_deg', 'yaw_harmonic_phase_deg']
LOW = np.array([2.5, 25, 25, 60, -10, -20, -15, -180, -10, 0, 0, -7, -3, -.5,
                -12, -180, 0, 0, .025, .025, -10, -180])
HIGH = np.array([5, 40, 60, 180, 15, 15, 15, 180, 10, 5, 5, 7, 3, .5,
                 12, 180, 1, 1, .45, .45, 10, 180])
CACHE = {}


def read(path):
    return json.loads(Path(path).read_text())


def vector(gait):
    g = asdict(Gait(**gait))
    return np.array([g[k] for k in KEYS])


def decode(v):
    g = dict(zip(KEYS, np.clip(v, LOW, HIGH).tolist()))
    g['roll_deg'] = min(g['roll_deg'], 40-abs(g['roll_bias_deg']))
    g['yaw_deg'] = min(g['yaw_deg'], 60-abs(g['yaw_bias_deg'])-abs(g['yaw_harmonic_deg']))
    for k in ['yaw_stroke_fraction', 'roll_stroke_fraction']:
        if g[k] < .15:
            g[k] = 0.
    return g


def evaluate(job):
    label, gait, feedback, seconds, dt = job
    config = read('configs/motors-speed-trained.json')
    config['timestep_s'] = dt
    key = json.dumps(config, sort_keys=True)
    if key not in CACHE:
        CACHE[key] = SwaySimulator('experiments/upstream-reachy-mini', config)
    g = Gait(**gait)
    period = round(50/g.frequency_hz)/50
    result = CACHE[key].run(g, cycles=math.ceil(seconds/period-1e-10), warmup_cycles=3,
                            feedback=feedback, sensor=read('configs/heading-sensor-assumed.json') if feedback else None)
    return dict(id=label, gait=gait, feedback=feedback, metrics=metrics(result), result=result)


def probes():
    base = read('configs/sway-speed-trained.json')
    rows = [base]
    for yaw in [37, 45, 52, 58]:
        for sharp in [.45, 1.5, 3.]:
            for phase in [105, 120, 135]:
                rows.append(dict(base, yaw_deg=yaw, yaw_sharpness=sharp, yaw_phase_deg=phase))
    for f in [3.85, 4.17, 5.]:
        for yaw in [45, 55]:
            for sharp in [.5, 2.]:
                rows.append(dict(base, frequency_hz=f, yaw_deg=yaw, yaw_sharpness=sharp))
    for width in [.4, .65, .85, 1.]:
        for yaw in [45, 55]:
            for roll in [0, .85]:
                rows.append(dict(base, yaw_deg=yaw, yaw_stroke_fraction=width,
                                 roll_stroke_fraction=roll, stroke_brake_fraction=.025))
    return [decode(vector(g)) for g in rows]


def flatten(report):
    return report.get('candidates', []) + [c for h in report.get('history', []) for c in h['candidates']]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['probe', 'search'], default='probe')
    p.add_argument('--input')
    p.add_argument('--allow-partial-seed', action='store_true', help='Recover completed generations from an interrupted seed report')
    p.add_argument('--output', required=True)
    p.add_argument('--generations', type=int, default=8)
    p.add_argument('--population', type=int, default=36)
    p.add_argument('--seconds', type=float, default=8.)
    p.add_argument('--seed', type=int, default=2026111921)
    args = p.parse_args()
    out = Path(args.output)
    if out.exists():
        p.error('Choose a new output path')
    if not 1 <= args.generations <= 16 or not 12 <= args.population <= 64 or not 4 <= args.seconds <= 20:
        p.error('Use bounded CPU exploration')
    feedback = read('configs/heading-speed-trained.json')
    report = dict(evidence='uncalibrated_simulation', hardware_ready=False, completed=False,
                  args=vars(args), physical_and_software_motor_limits_unchanged=True,
                  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in
                                 [Path(__file__), Path('src/reachy_locomotion/sway.py'),
                                  Path('src/reachy_locomotion/heading.py'), Path('src/reachy_locomotion/motors.py'),
                                  Path('configs/motors-speed-trained.json')]}, candidates=[], history=[])
    out.parent.mkdir(parents=True, exist_ok=True)
    def save():
        out.write_text(json.dumps(report, allow_nan=False)+'\n')
    rng = np.random.default_rng(args.seed)
    with ProcessPoolExecutor(max_workers=4) as pool:
        if args.mode == 'probe':
            jobs = [(f'probe-{i}',g,feedback,args.seconds,.001) for i,g in enumerate(probes())]
            for r in pool.map(evaluate, jobs):
                report['candidates'].append(r); save()
                print(json.dumps(dict(id=r['id'], **r['metrics'])), flush=True)
        else:
            if not args.input:
                p.error('Search requires a completed seed report')
            previous = read(args.input)
            if not previous['completed'] and not args.allow_partial_seed:
                p.error('Wait for the seed experiment to finish')
            report['seed_report_completed'] = previous['completed']
            seeds = sorted(flatten(previous), key=lambda r:r['metrics']['score'], reverse=True)
            # Deduplicate numerical gaits, retaining high-scoring independent basins.
            seeds = list({json.dumps(r['gait'],sort_keys=True):r for r in seeds}.values())[:16]
            best = seeds[0]; mean = vector(best['gait']); std = .045*(HIGH-LOW)
            for gen in range(args.generations):
                rows = rng.normal(mean, std, size=(args.population,len(KEYS)))
                rows[0] = vector(best['gait'])
                rows[1] = vector(read('configs/sway-speed-trained.json'))
                for i in range(2, args.population):
                    if i % 3:
                        centre = best if i % 2 else seeds[(gen+i) % min(8,len(seeds))]
                        rows[i] = rng.normal(vector(centre['gait']), (.012 if i % 4 == 1 else .04)*(HIGH-LOW))
                    # Include large swings and short brakes without relaxing bounds.
                    if i >= args.population-4:
                        rows[i] = vector(best['gait'])
                        rows[i,KEYS.index('yaw_deg')] = rng.uniform(48,59)
                        rows[i,KEYS.index('yaw_stroke_fraction')] = rng.uniform(.4,1)
                        rows[i,KEYS.index('stroke_brake_fraction')] = rng.uniform(.025,.07)
                    elif i >= args.population-8:
                        rows[i,KEYS.index('yaw_deg')] += rng.uniform(3,10)
                        rows[i,KEYS.index('yaw_sharpness')] += rng.uniform(.5,2)
                jobs = [(f'g{gen}-{i}',decode(v),feedback,args.seconds,.001) for i,v in enumerate(rows)]
                candidates = sorted(pool.map(evaluate,jobs), key=lambda r:r['metrics']['score'], reverse=True)
                if candidates[0]['metrics']['score'] > best['metrics']['score']:
                    best = candidates[0]
                elites = np.array([vector(r['gait']) for r in candidates[:6]])
                mean = .2*mean+.8*elites.mean(axis=0)
                std = np.maximum(.3*std+.7*elites.std(axis=0), .008*(HIGH-LOW))
                seeds = sorted(seeds+candidates[:6],key=lambda r:r['metrics']['score'],reverse=True)[:16]
                report['history'].append(dict(generation=gen,candidates=candidates))
                report['best_short'] = best; save()
                print(json.dumps(dict(generation=gen, **best['metrics'],gait=best['gait'])), flush=True)
    report['completed'] = True; save()


if __name__ == '__main__':
    main()
