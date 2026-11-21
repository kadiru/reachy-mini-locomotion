"""Search a one-stroke-per-second walk with sensor-triggered yaw and braking."""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from train_cadence import read, rollout, save


def quality(row, both_edges=False):
    feasible = row['completed'] and row['settled'] and row['cadence_ok'] and row['course_ok']
    brakes = len(row['edge_brake_strokes'])
    bilateral = both_edges and all(row.get('edge_brake_by_roll_sign', {}).get(s) for s in ('left','right'))
    return (feasible, row['passed'] and bilateral, row['passed'] and brakes >= 2, row['passed'] and brakes >= 1,
            row['mm_per_swing'] + .28 * min(row['rim_lift_mm'], 13) + .35 * brakes)


def gate(rng):
    return dict(delay_ticks=1, lead_s=float(rng.uniform(.015, .09)),
                launch_deg=float(rng.choice([.12, .4, .9, 1.8])), max_wait_s=float(rng.uniform(.02, .14)),
                stop_deg=float(rng.uniform(1.4, 6)), min_fraction=float(rng.uniform(.32, .8)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--workers', default=4, type=int)
    parser.add_argument('--refine', type=Path, help='Refine completed forward/edge candidates from a previous run')
    parser.add_argument('--encoder-catch', action='store_true', help='Train encoder-referenced braking after the yaw command finishes')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    prior = read('experiments/reports/motor-training-teleop-cadence-2026-11-20.json')
    cfg = read('configs/motors-speed-trained.json')
    seeds = [prior['experiments']['forward']['candidates'][key]
             for key in ('lift-2-3', 'lift-2-10', 'lift-2-4', 'lift-0-28')]
    seeds.append(prior['backward_edge_example']['policy'])
    if args.encoder_catch and not args.refine:
        parser.error('--encoder-catch requires a previous search')
    rng = np.random.default_rng(2026112191 if args.encoder_catch else 2026112161 if args.refine else 2026112141)
    candidates, history = {}, {}
    def score(row):
        rank = quality(row, both_edges=bool(args.refine))
        if args.encoder_catch:
            active = bool(row.get('sensor_stop_strokes'))
            return (rank[0], row['passed'] and active and rank[1],
                    row['passed'] and active and bool(row['edge_brake_strokes']), *rank[1:])
        return rank
    def jobs(names, sides, seconds, dt):
        return [(name, candidates[name], side, seconds, dict(cfg, timestep_s=dt), None)
                for name in names for side in sides]

    def batch(label, trials):
        rows = list(pool.map(rollout, trials))
        history[label] = rows
        save(args.out/'history.json', history)
        best = max(rows, key=score)
        print(label, len(rows), best['name'], round(best['distance_mm'], 2),
              best['edge_brake_strokes'], flush=True)
        return rows

    def rank_group(name, rows):
        return min(score(row) for row in rows if row['name'] == name)

    bounds = {'pitch':(-12,15,2.8), 'x':(-18,20,3.), 'x_arc':(-14,16,3.),
              'y':(-25,25,1.6), 'roll':(19,40,1.8), 'yaw':(-48,48,3.),
              'y_s':(.045,.24,.012), 'yaw_s':(.055,.24,.018),
              'roll_s':(.045,.24,.012), 'yaw_delay':(0,.22,.022),
              'y_delay':(0,.2,.014), 'roll_delay':(0,.2,.012)}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        if args.refine:
            previous = read(args.refine/'candidates.json')
            paired = read(args.refine/'history.json')['bilateral']
            near = [r for r in paired if r['completed'] and r['settled'] and r['edge_brake_strokes']]
            near.sort(key=lambda r:(len(r['edge_brake_strokes']), r['mm_per_swing']), reverse=True)
            names = list(dict.fromkeys(r['name'] for r in sorted(near,key=quality,reverse=True)[:3]+near[:3]))
            seeds = [previous[n] for n in names]
        parents = seeds
        for gen in range(3):
            names = []
            for i in range(40):
                parent = parents[i % len(parents)]
                p = dict(parent, swings_per_block=8)
                if args.refine:
                    if i >= len(parents):
                        for key in rng.choice(list(bounds), size=3, replace=False):
                            lo, hi, sigma = bounds[key]
                            p[key] = float(np.clip(rng.normal(p[key],sigma*.65),lo,hi))
                    p['edge_gate'] = dict(parent['edge_gate'])
                    if i >= len(parents) and i % 3 == 1:
                        p['edge_gate']['stop_deg'] *= float(rng.uniform(.8,1.25))
                    if args.encoder_catch and ('catch_deg' not in p['edge_gate'] or i >= len(parents)):
                        p['edge_gate'].update(catch_deg=float(rng.uniform(.4,4.5)),
                                              catch_range_deg=float(rng.uniform(2.5,9.)))
                elif gen or i >= len(seeds)*3:
                    for key, (lo, hi, sigma) in bounds.items():
                        p[key] = float(np.clip(rng.normal(p[key],sigma), lo, hi))
                if not args.refine:
                    p['edge_gate'] = gate(rng)
                    if gen and i % 3:
                        p['edge_gate'] = dict(parent['edge_gate'], stop_deg=p['edge_gate']['stop_deg'])
                name = f'g{gen}-{i}'
                candidates[name] = p
                names.append(name)
            save(args.out/'candidates.json', candidates)
            rows = batch(f'search-{gen}', jobs(names, [1 if gen != 1 else -1], 8, .0005 if args.refine else .001))
            ranked = sorted(rows, key=score, reverse=True)
            near = sorted((r for r in rows if r['passed']), key=lambda r:r['rim_lift_mm'], reverse=True)
            parents = [candidates[n] for n in dict.fromkeys(r['name'] for r in ranked[:5]+near[:3])]

        finalists = list(dict.fromkeys(r['name'] for gen in range(3)
                         for r in sorted(history[f'search-{gen}'], key=score, reverse=True)[:4]))
        paired = batch('bilateral', jobs(finalists, [-1,1], 12, .0005))
        finalists = sorted(finalists, key=lambda n:rank_group(n,paired), reverse=True)[:3]
        fine = batch('nominal', jobs(finalists, [-1,1], 24, .00025))
        winner = max(finalists, key=lambda n:rank_group(n,fine))
        p = candidates[winner]
        save(args.out/'policy.json', dict(p, selected_candidate=winner, hardware_ready=False))
        stress = np.random.default_rng(2026112192 if args.encoder_catch else 2026112162 if args.refine else 2026112142)
        trials, conditions = [], []
        for i in range(5):
            c = copy.deepcopy(cfg)
            c['timestep_s'] = .0005
            c['friction_slide_torsion_roll'][0] = float(stress.uniform(.55,.9))
            c['motor_model'].update(voltage_v=float(stress.uniform(4.7,5.5)), pid_scale=float(stress.uniform(.9,1.1)),
                                    delay_frames=int(stress.integers(0,3)), mass_distribution=['uniform','torso'][i%2])
            conditions.append(c)
            trials.extend((f'floor-{i}',p,side,16,c,None) for side in (-1,1))
        save(args.out/'heldout-conditions.json', conditions)
        batch('heldout', trials)
        batch('finer', jobs([winner], [-1,1], 24, .000125))
        candidates['timed-only'] = {key:value for key,value in p.items() if key != 'edge_gate'}
        batch('timed-control', jobs(['timed-only'], [-1,1], 24, .00025))
        if args.encoder_catch:
            candidates['sensors-muted'] = dict(p, edge_gate=dict(p['edge_gate'], launch_deg=1e6, stop_deg=1e6, catch_deg=1e6))
            batch('muted-control', jobs(['sensors-muted'], [-1,1], 24, .00025))
        batch('record', [(winner,p,1,24,dict(cfg,timestep_s=.00025),str(args.out/'walk.json'))])


if __name__ == '__main__':
    main()
