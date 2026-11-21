"""Fit a stock-rate sensor proxy on unstruck, natural-start side-edge holds."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_balance_robustness import evaluate, read
from train_swing_distance import save


def trial(spec):
    edge, seed, weight, averaged, noise = spec
    p = read('configs/balance-recovery-experimental.json')['edges'][edge]
    p['feedback'].update(sensor_seed=seed, sensor_noise_deg=noise)
    p['sensors'].update(gyro_interval_average=averaged, attitude_rate_weight=weight)
    result = evaluate(dict(id=edge, edge=edge, policy=p, dt=.000125, seconds=14))
    metrics, run = result['metrics'], result['result']
    return dict(edge=edge, seed=seed, weight=weight, averaged=averaged, attitude_noise_deg=noise,
                final_hold=metrics['final_five_seconds_pass'], longest_s=metrics['longest_strict_hold_s'],
                termination=run['termination'], peak_tilt_deg=run['peak_tilt_deg'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Choose a new output path')
    report = dict(evidence='Simulated delayed noisy base attitude, ideal relative head pose, one gyro sample per 20 ms',
                  duration_s=14, dt=.000125, hardware_ready=False)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        edges = ('left','right')
        weights = (0., .25, .5, .75, .9, .98, 1.)
        jobs = [(edge, 2026112101, w, False, .05) for w in weights for edge in edges]
        fit = report['fit'] = list(pool.map(trial, jobs))
        def rank(weight):
            rows = [r for r in fit if r['weight'] == weight]
            return sum(r['final_hold'] for r in rows), min(r['longest_s'] for r in rows), sum(r['longest_s'] for r in rows)
        chosen = report['selected_weight'] = max(weights, key=rank)
        print('fit', chosen, rank(chosen), flush=True)
        save(args.out, report)
        # These seeds are absent from fitting. Include matched positive and negative controls.
        heldout = [(e,s,w,a,.05) for s in (2026112151,2026112152,2026112153) for e in edges
                   for w,a in dict.fromkeys([(chosen,False),(0.,False),(0.,True)])]
        report['heldout'] = list(pool.map(trial, heldout))
        report['noise_sensitivity'] = list(pool.map(trial, [(e,2026112154,chosen,False,n)
                                                      for n in (.025,.1,.2) for e in edges]))
        save(args.out, report)


if __name__ == '__main__':
    main()
