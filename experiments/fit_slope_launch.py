"""Search the length, strength and braking point of successive lateral head strokes."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from slope_balance import WORK, run, spec, read, write, grade


def sequence(rng):
    half=float(rng.uniform(.115,.30))
    d=np.clip(half*np.linspace(rng.uniform(.55,1.1),rng.uniform(.8,1.8),10)*rng.uniform(.8,1.2,10),.08,.55)
    d[0]=rng.uniform(.15,.48)
    return dict(durations=d.tolist(),amp=float(rng.uniform(25,40)),y=float(rng.uniform(-24,10)),
                bias=float(rng.uniform(-8,8)),start_sign=int(rng.choice([-1,1])),z=float(rng.uniform(-12,8)),
                pitch=float(rng.uniform(-6,6)),x=float(rng.uniform(-8,8)),transition=float(rng.uniform(.04,.16)),
                min_trigger=float(rng.uniform(15,26)),lead=float(rng.uniform(.01,.15)),trigger=float(rng.uniform(26,34)),
                catch_model=str(rng.choice(['full','plane'])))


def merit(r):
    return grade(r)-.22*r.get('capture_distance',max(0.,32-r['peak_requested_lean']))


def sweep(pool):
    rng=np.random.default_rng(60870)
    candidates=[sequence(rng) for _ in range(75)]
    rows=[]
    for bank in (5.,-5.,0.,9.,-9.):
        jobs=[spec(bank,s,p,seconds=2.8,dt=.00025) for s in (1,-1) for p in candidates]
        rows.extend(dict(settings=j[2],result=r) for j,r in zip(jobs,pool.map(run,jobs)))
        write(WORK/'strokes-progress.json',rows)
    return rows


def validate(pool):
    all_rows=read(WORK/'search.json')+read(WORK/'strokes.json')
    jobs=[]
    for bank in (0.,5.,-5.,9.,-9.):
        for side in (-1,1):
            here=[row for row in all_rows if row['result']['bank']==bank and row['result']['side']==side]
            mirror=[row for row in all_rows if row['result']['bank']==-bank and row['result']['side']==-side]
            options=[v['settings'] for v in sorted(here,key=lambda q:merit(q['result']),reverse=True)[:5]]
            options+=[v['settings'] for v in sorted(mirror,key=lambda q:merit(q['result']),reverse=True)[:2]]
            jobs.extend(spec(bank,side,p,seconds=5.,dt=.000125) for p in options)
    return [dict(settings=j[2],result=r) for j,r in zip(jobs,pool.map(run,jobs))]


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('task',choices=['strokes','validate'])
    a=ap.parse_args()
    with ProcessPoolExecutor(6) as pool:
        write(WORK/(a.task+'.json'),(sweep if a.task=='strokes' else validate)(pool))
