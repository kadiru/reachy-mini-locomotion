"""Tune pair-end heading correction while preserving slow, alternating rim brakes."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_cadence import read,rollout,save


def rank(r):
    rims = r['edge_brake_by_roll_sign']
    return (r['passed'], all(rims.values()) and len(r['edge_brake_strokes'])>=r['planned_swings']/2,
            all(rims.values()), r['mm_per_swing'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=3)
    a = ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    old = read(a.source/'candidates.json')
    nominal = read(a.source/'history.json')['nominal']
    names = list(dict.fromkeys(r['name'] for r in nominal))
    cfg = read('configs/motors-speed-trained.json')
    policies, history = {}, {}
    for n in names:
        for k,(bias,delay) in enumerate([(0.,0.),(-.012,0.),(.012,0.),(-.03,0.),(.03,0.),
                                         (0.,-.003),(0.,.003),(0.,-.009),(0.,.009)]):
            policies[f'{n}-{k}'] = dict(old[n],pair_steering=dict(yaw_bias_per_deg=bias,delay_s_per_deg=delay))
    save(a.out/'candidates.json',policies)
    def batch(label,names,sides,dt):
        rows = list(pool.map(rollout,[(n,policies[n],s,24,dict(cfg,timestep_s=dt),None) for n in names for s in sides]))
        history[label]=rows
        save(a.out/'history.json',history)
        return rows
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        rows = batch('fit',policies,[1],.0005)
        top = [r['name'] for r in sorted(rows,key=rank,reverse=True)[:6]]
        # Retain the best candidate with active brakes as well as the fastest overall.
        live = [r for r in rows if r['sensor_stop_strokes']]
        top = list(dict.fromkeys(top+[r['name'] for r in sorted(live,key=rank,reverse=True)[:3]]))
        rows = batch('nominal',top,[-1,1],.00025)
        win = max(top,key=lambda n:min(rank(r) for r in rows if r['name']==n))
        save(a.out/'policy.json',dict(policies[win],selected_candidate=win,hardware_ready=False))
        print(win, [(r['side'],r['passed'],round(r['distance_mm'],2)) for r in rows if r['name']==win],flush=True)


if __name__=='__main__':
    main()
