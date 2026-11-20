"""Freeze and qualify the progress-adaptive bilateral controller; never tune on these results."""
import argparse
import copy
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

from reachy_locomotion.balance import mirrored
from reachy_locomotion.penguin import PenguinController
from reachy_locomotion.primitives import PrimitiveSimulator
from train_balance import models
from train_penguin import metrics
from qualify_primitives import pivot_metrics
from finalize_primitives import record_terminal

CACHE = {}

def evaluate(job):
    p = job['policy']; config = job['config']; side = job['side']; pivot = job['pivot_only']
    key = json.dumps(config, sort_keys=True)
    if key not in CACHE:
        CACHE[key] = PrimitiveSimulator('experiments/upstream-reachy-mini', config)
    sim = CACHE[key]; g = p['primary']['recipe']; f = p['primary']['feedback']
    ctl = PenguinController(g, f, cycles=job.get('cycles', 1), min_step_s=3., start_side=side,
                            checked_ik=sim.checked_ik, target_step_turn_deg=p['target_step_turn_deg'],
                            max_repeats=p['max_repeats'], pivot_only=pivot,
                            heading_noise_deg=job.get('heading_noise_deg', .1), heading_seed=job.get('heading_seed', 42),
                            fallback_policy=p['fallback'], switch_below_deg=p['switch_below_deg'])
    r = sim.run_primitive(mirrored(g, side), seconds=job.get('seconds', 15 if pivot else 28),
                          waist_mode='hold_neutral', controller=ctl, tilt_limit_deg=35, record=job.get('record', False))
    summary = ctl.report(); pm = pivot_metrics(r)
    if not pivot:
        summary.update(metrics(r, r['trace'], summary))
    passed = (summary['completed'] and pm['passed'] and -side*r['final_foot_yaw_deg'] >= 5
              and -side*r['edge_supported_foot_yaw_deg'] >= 5) if pivot else summary['passed']
    if job.get('record'):
        record_terminal(sim, r)
        r.update(penguin=summary, balance_policy=p, render_forward_frame='startup')
        path = Path(job['record']); path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            raise FileExistsError(path)
        path.write_text(json.dumps(r, allow_nan=False) + '\n')
    r.pop('trace'); r.pop('terminal_state'); r.pop('trajectory', None)
    return dict(id=job['id'], side=side, pivot_only=pivot, result=r, penguin=summary, pivot_metrics=pm, passed=passed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--policy', default='configs/penguin-feedback-experimental.json')
    parser.add_argument('--seed', type=int, default=2026111971)
    args = parser.parse_args(); out = Path(args.output)
    if out.exists():
        parser.error('Preserve previous evidence')
    policy = json.loads(Path(args.policy).read_text()); jobs = []
    def append(label, config, p, side, seed):
        for pivot in [True, False]:
            jobs.append(dict(id=label, config=config, policy=p, side=side, pivot_only=pivot, heading_seed=seed))
    for i, config in enumerate(models()):
        config = copy.deepcopy(config); config['timestep_s'] = .00025
        for side in [-1, 1]:
            append(f'development-{i}', config, policy, side, 42)
    rng = np.random.default_rng(args.seed)
    for i in range(8):
        config = copy.deepcopy(models()[0]); config['timestep_s'] = .00025
        config['friction_slide_torsion_roll'][0] = float(rng.uniform(.55, .9))
        config['motor_model'].update(voltage_v=float(rng.uniform(4.7, 5.5)), pid_scale=float(rng.uniform(.9, 1.1)),
                                    delay_frames=int(rng.integers(0, 3)), mass_distribution=['torso', 'uniform'][i%2])
        for side in [-1, 1]:
            p = copy.deepcopy(policy)
            sensor = dict(sensor_delay_frames=int(rng.integers(1, 3)), sensor_noise_deg=float(rng.uniform(.05, .15)),
                          sensor_seed=int(rng.integers(0, 100000)))
            for name in ['primary', 'fallback']:
                p[name]['feedback'].update(sensor)
            append(f'fresh-{i}', config, p, side, int(rng.integers(0, 100000)))
    for side in [-1, 1]:
        config = copy.deepcopy(models()[0]); config['timestep_s'] = .000125
        append('nominal-0.125ms', config, policy, side, 42)
    report = dict(evidence='uncalibrated_simulation', hardware_ready=False, completed=False, seed=args.seed,
                  frozen_policy=policy, frozen_jobs=jobs, checks=[])
    def save():
        out.write_text(json.dumps(report, allow_nan=False) + '\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate, jobs):
            report['checks'].append(c); save()
            print(json.dumps(dict(id=c['id'], side=c['side'], pivot=c['pivot_only'], passed=c['passed'],
                                  stable=c['result']['recovered'], termination=c['result']['termination'],
                                  bursts=len(c['penguin']['bursts']), yaw=c['result']['final_foot_yaw_deg'],
                                  stride=c['penguin'].get('mean_stride_mm'))), flush=True)
    report['completed'] = True; save()

if __name__ == '__main__':
    main()
