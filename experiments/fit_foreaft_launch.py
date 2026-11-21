"""Search for a natural pitch launch into the existing front/back regulators."""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from train_balance_robustness import evaluate, read
from train_swing_distance import save


def trial(spec, recording=None):
    edge, params, seed, dt, seconds = spec
    bank = read('configs/balance-recovery-experimental.json')
    p = copy.deepcopy(bank['capture_policies'].get(edge, bank['edges'][edge]))
    p['feedback'].update(sensor_seed=seed, capture_interpolate=True,
                         prediction_s=params['prediction_s'], trigger_deg=params['trigger_deg'],
                         capture_blend_s=params['blend_s'])
    p['recipe'].update(pump_hz=params['pump_hz'], pump_y_phase_deg=params['phase_deg'])
    r = evaluate(dict(id=edge,edge=edge,policy=p,dt=dt,seconds=seconds,observe_landing=True,
                      record=recording))
    m, run = r['metrics'], r['result']
    assert run['natural_upright_start'] and run['commanded_waist_position_range_rad'] == [0.,0.]
    return dict(edge=edge, params=params, seed=seed, dt=dt, seconds=seconds,
                held=m['final_five_seconds_pass'], strict_s=m['longest_strict_hold_s'], soft_s=m['soft_hold_s'],
                peak_pitch_deg=m['peak_sampled_pitch_deg'], final_pitch_deg=m['final_pitch_deg'],
                first_floor_contact=run['first_body_floor_contact'], termination=run['termination'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2)
    a = parser.parse_args()
    if a.out.exists():
        parser.error('Choose a new output path')
    rng = np.random.default_rng(2026112181)
    jobs = []
    for edge in ('front','back'):
        for i in range(16):
            params = dict(pump_hz=float(rng.uniform(2.55,3.2)), phase_deg=float(rng.uniform(-85,-30)),
                          trigger_deg=float(rng.uniform(22,31.5)), blend_s=float(rng.uniform(.025,.18)),
                          prediction_s=float(rng.uniform(0,.095)))
            jobs.append((edge,params,2026112101,.00025,9))
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        fit = list(pool.map(trial,jobs))
        report = dict(evidence='MuJoCo only, flat starts, fixed waist, physical landing observed', fit=fit)
        save(a.out, report)
        tests = []
        for edge in ('front','back'):
            winner = max((r for r in fit if r['edge']==edge),key=lambda r:(r['held'],r['strict_s'],r['soft_s']))
            tests.extend((edge,winner['params'],s,.000125,14) for s in (2026112182,2026112183))
        report['confirmation'] = list(pool.map(trial,tests))
        save(a.out, report)


if __name__ == '__main__':
    main()
