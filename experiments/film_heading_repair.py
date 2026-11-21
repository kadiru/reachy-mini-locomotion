"""Replay every post-repair walking qualification and film both starting sides."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from film_hit_envelope import film
from train_cadence import read,rollout


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=4)
    ap.add_argument('--record-only',action='store_true')
    a = ap.parse_args()
    a.work.mkdir(parents=True,exist_ok=True)
    data = read('experiments/reports/motor-training-cadence-heading-repair-2026-11-21.json')
    old = read('experiments/reports/motor-training-alternating-edge-walks-2026-11-21.json')
    cases,missing = {},[]
    for label,conditions in [('retest',old['conditions']),('fresh',data['fresh_conditions']),
                             ('long',[dict(read('configs/motors-speed-trained.json'),timestep_s=.00025)])]:
        for i,r in enumerate(data[label]):
            c = conditions[i//2]
            path = a.work/f'{label}-{i}.json'
            if not path.exists():
                missing.append((r['name'],data['policy'],r['side'],r['active_seconds'],c,str(path),True))
            cases[label,i] = (path,r,c)
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        list(pool.map(rollout,missing))
    chapters = []
    for floor in range(6):
        panels = []
        for label in (['retest','fresh'] if floor<5 else ['long']):
            for start in range(2):
                path,r,c = cases[label,2*floor+start if floor<5 else start]
                replay=read(path)['cadence_result']
                if abs(replay['distance_mm']-r['distance_mm'])>1e-7 or replay['passed']!=r['passed']:
                    raise ValueError(f'Replay mismatch in {path}')
                panels.append(dict(record=path,label=f"{label.capitalize()}   ·   start {r['side']:+d}",
                                   outcome=f"{r['distance_mm']:+.1f} mm   ·   pair-end {r['max_pair_end_deg']:.1f}°   ·   "+('pass' if r['passed'] else 'fail'),
                                   detail=f"{c['motor_model']['voltage_v']:.2f} V   ·   friction {c['friction_slide_torsion_roll'][0]:.2f}   ·   delay {c['motor_model']['delay_frames']} frames",
                                   color='#ade1ba' if r['passed'] else '#ffad92'))
        chapters.append(dict(title=f'Floor set {floor+1}' if floor<5 else 'Forty-second nominal walk',
                             subtitle='Repaired heading feedback   ·   60 directional swings/min   ·   fixed waist',panels=panels))
    if not a.record_only:
        film(chapters,a.output,a.work/'chapters',speed=2,fps=15)


if __name__=='__main__':
    main()
