"""Enumerate whole-pair budgets and measure the fewest swings to stop past a target."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_swing_distance import read, save, trial


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--target-mm',type=float,default=100.)
    args=parser.parse_args()
    if args.output.exists() or args.target_mm<=0:
        parser.error('Choose a new report path and a positive distance')
    candidates={'trained':read(args.policy)}
    for name,tag in [('rocking','rocking-experimental'),('rapid','faster-trained')]:
        candidates[name]=dict(gait=read(f'configs/sway-{tag}.json'),feedback=read(f'configs/heading-{tag}.json'))
    config=dict(read('configs/motors-speed-trained.json'),timestep_s=.00025)
    sensor=read('configs/heading-sensor-assumed.json')
    jobs=[(n,p,config,s,b,sensor,None) for n,p in candidates.items() for b in range(1,15) for s in (-1,1)]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(trial,jobs))
    winners=[]
    for name in candidates:
        for side in (-1,1):
            possible=[r for r in rows if r['name']==name and r['side']==side and r['passed'] and r['distance_mm']>=args.target_mm]
            if possible:
                winners.append(min(possible,key=lambda r:(r['counts']['10']['charged'],r['pairs'])))
    save(args.output,dict(target_mm=args.target_mm,whole_pair_budgets=list(range(1,15)),
                          conditions='nominal 5 V, 0.25 ms, both starting sides, from rest to rest',
                          winners=winners,trials=rows))
    for r in winners:
        print(r['name'],r['side'],r['pairs'],r['counts']['10']['charged'],round(r['distance_mm'],2))


if __name__=='__main__':
    main()
