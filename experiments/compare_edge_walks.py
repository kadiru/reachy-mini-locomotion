"""Compare frozen walking policies under the same previously unused conditions."""
import argparse
import copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from train_cadence import read, rollout, save


def select(folder, require_feedback, repeated_alternation=False):
    rows = read(folder/'history.json')['nominal']
    candidates = read(folder/'candidates.json')
    def rank(name):
        group = [r for r in rows if r['name']==name]
        if {r['side'] for r in group} != {-1,1}:
            raise ValueError('Both starts required')
        def quality(r):
            by_side = r['edge_brake_by_roll_sign']
            brakes = len(r['edge_brake_strokes'])
            feedback = bool(r['sensor_stop_strokes']) or not require_feedback
            sustained = min(map(len,by_side.values())) >= .4*r['planned_swings']
            return (r['passed'], feedback, repeated_alternation and sustained,
                    all(by_side.values()) and brakes >= r['planned_swings']/2,
                    all(by_side.values()), brakes >= 2, r['mm_per_swing'])
        return min(quality(r) for r in group)
    winner = max(dict.fromkeys(r['name'] for r in rows), key=rank)
    return dict(candidates[winner], selected_candidate=winner, hardware_ready=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timed', type=Path, required=True)
    parser.add_argument('--encoder', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--states', type=Path, help='Save every matched test, including failures, for film review')
    parser.add_argument('--alternating-only',action='store_true',
                        help='Select for repeated braking on each rim and test on the same frozen conditions')
    parser.add_argument('--workers', type=int, default=4)
    a = parser.parse_args()
    if a.out.exists():
        parser.error('Choose a new report path')
    if a.states:
        a.states.mkdir(parents=True,exist_ok=False)
    base = read('configs/motors-speed-trained.json')
    policies = dict(shallow=read('configs/teleop-cadence-trained.json'),timed=select(a.timed,False),
                    encoder=select(a.encoder,True))
    if a.alternating_only:
        policies = {'alternating':select(a.timed,False,repeated_alternation=True)}
    print('frozen from nominal', {k:p['selected_candidate'] for k,p in policies.items()}, flush=True)
    rng = np.random.default_rng(2026112199)
    conditions, jobs = [], []
    for number in range(5):
        model = copy.deepcopy(base)
        model['timestep_s'] = .0005
        model['friction_slide_torsion_roll'][0] = float(rng.uniform(.55,.9))
        model['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),
                                    delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][number%2])
        conditions.append(model)
        jobs.extend((f'{name}-{number}',policy,side,16,model,
                     str(a.states/f'{name}-{number}-{side}.json') if a.states else None,True)
                    for name,policy in policies.items() for side in (-1,1))
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        rows = list(pool.map(rollout,jobs))
        checks = list(pool.map(rollout, [(name,p,s,24,dict(base,timestep_s=.000125),None)
                                         for name,p in policies.items() if name!='shallow' for s in (-1,1)]))
        control = []
        if 'encoder' in policies:
            p = policies['encoder']
            muted = dict(p,edge_gate=dict(p['edge_gate'],launch_deg=1e6,stop_deg=1e6,catch_deg=1e6))
            control = list(pool.map(rollout,[('muted',muted,s,24,dict(base,timestep_s=.00025),None) for s in (-1,1)]))
    save(a.out, dict(evidence='MuJoCo only',seed=2026112199,policies=policies,conditions=conditions,
                     rows=rows,finer=checks,muted_control=control))


if __name__ == '__main__':
    main()
