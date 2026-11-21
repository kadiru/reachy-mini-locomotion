"""Fit a head-command tracking limit against previously failed direct side hits."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_balance_robustness import read, evaluate
from train_swing_distance import save


def trial(spec):
    edge, source, mass, speed, limit, seed, release = spec
    bank = read('configs/balance-recovery-experimental.json')
    p = bank['edges'][edge]
    p['disturbance_tracking_limit'] = limit
    p['feedback']['sensor_seed'] = seed
    ball = dict(bank['rolling_ball'], launch_s=release, speed_m_s=speed,
                from_side=1 if source=='left' else -1, mass_kg=mass,
                launch_height_m=.18, distance_m=.30 if edge==source else .21, vertical_speed_m_s=0.)
    r = evaluate(dict(id=edge,edge=edge,policy=p,dt=.000125,seconds=14,balls=[ball]))
    run = r['result']
    event = run['ball_disturbances'][0]
    hit, floor = event['first_robot_contact_s'],event['first_floor_contact_s']
    direct = hit is not None and hit>release+.00025 and (floor is None or hit<floor)
    m = r['impacts']['events'][0]
    return dict(edge=edge,source=source,mass_kg=mass,speed_m_s=speed,limit=limit,seed=seed,
                release_s=release,direct=direct,qualified=m['balanced_before_impact'],rebounded=m['rebounded'],
                passed=bool(direct and run['balance_curriculum']['impact_trial_pass'] and r['metrics']['final_five_seconds_pass']),
                recovery_s=m['recovery_s'], final_hold=r['metrics']['final_five_seconds_pass'],
                incoming_speed_m_s=event['incoming_3d_speed_m_s'], termination=run['termination'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=3)
    a = parser.parse_args()
    if a.out.exists():
        parser.error('Choose a new output path')
    edges = ('left','right')
    limits = (0.,3.,8.,16.,30.)
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        fit = list(pool.map(trial, [(e,s,.010,1.4,w,2026112101,5.) for e in edges for s in edges for w in limits]))
        def rank(w):
            rows = [r for r in fit if r['limit']==w]
            return (sum(r['passed'] for r in rows),sum(r['qualified'] for r in rows),
                    sum(r['recovery_s'] is not None for r in rows))
        winner = max(limits,key=rank)
        report = dict(evidence='MuJoCo only, training gyro averaging, fixed waist', fit=fit, selected_limit=winner)
        save(a.out, report)
        # Keep the baseline on ties, and test the leading nonzero setting as a labelled challenger.
        challenger = max(limits[1:],key=rank)
        report['challenger'] = challenger
        report['heldout'] = list(pool.map(trial, [(e,s,m,v,w,2026112231,6.) for e in edges for s in edges
                                              for m,v in [(.005,1.),(.010,1.4),(.005,2.)] for w in (0.,challenger)]))
        save(a.out, report)


if __name__ == '__main__':
    main()
