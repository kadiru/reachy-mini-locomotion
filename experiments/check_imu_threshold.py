"""Challenge the apparent high-rate threshold at independent phases and release states."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json

from imu_frequency_study import WORK
from multirate_imu import feed_archived_runner
from train_imu_catch import run, PARAMS
from moving_table import read


def trial(job):
    policy,rate,repeat,band = job
    phase = (.1,.3,.65,.9)[repeat % 4]
    seed = 51631 + repeat
    angle, speed = (32.2,0.) if repeat < 4 else ((32.15,.2) if repeat%2==0 else (32.25,-.2))
    with feed_archived_runner(rate,phase,band=band):
        r = run((read(PARAMS)['parameters'] if policy=='refit' else None,
                 (angle,speed,seed,10*phase),8.),audit_contact=True)
    return dict(policy=policy,rate=rate,repeat=repeat,phase=phase,band=band,angle=angle,
                speed=speed,seed=seed,held=r['held'],flat=r['flat'],shell=bool(r['shell']),
                final_roll_deg=r['final_roll'],support_fraction=r['support_fraction'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=3)
    args=parser.parse_args()
    jobs = [('original',hz,i,'wide') for hz in ('stock',50,400,500,1000) for i in range(8)]
    jobs += [('refit',hz,i,band) for hz in (50,500) for band in ('wide','narrow') for i in range(4)]
    out = WORK/'threshold.jsonl'
    done = {tuple(json.loads(l)[k] for k in ('policy','rate','repeat','band')) for l in
            out.read_text().splitlines()} if out.exists() else set()
    with ProcessPoolExecutor(args.workers) as pool, out.open('a') as f:
        for row in pool.map(trial,[j for j in jobs if j not in done]):
            print(json.dumps(row,allow_nan=False),file=f,flush=True)

if __name__=='__main__':
    main()
