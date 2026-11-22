"""Measure displacement of the supporting contact in coordinates fixed to the moving deck."""
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor

from moving_table import REPORT, SEEDS, SLOW, case, key, read, simulate, write
from qualify_moving_table import ORIENTATIONS, bounds, edge_pass

FOOTPRINT = REPORT.with_name('moving-table-footprint.json')


def evaluate(job):
    row = simulate(job, measure_footprint=True)
    row['held_in_place'] = bool(edge_pass(row['result']) and row['max_footprint_travel_mm'] <= 10.)
    return row


def qualify(workers=4, source=REPORT):
    study = read(source)
    limits = {(s['side'], 1 if s['orientation']=='downhill' else -1): s['angle_pass'] for s in study['summary']}
    grades = {sd: sorted({0.,limits[sd],*[a for a in (2.,5.,10.,15.,20.,25.) if a<limits[sd]]}) for sd in ORIENTATIONS}
    evidence = {}
    outcomes = defaultdict(dict)
    with ProcessPoolExecutor(workers) as pool:
        def run(specs):
            jobs=[case(s,s*d*a,SLOW,seed) for (s,d),a in specs for seed in SEEDS]
            new = {key(job): job for job in jobs if key(job) not in evidence}
            for identifier, row in zip(new, pool.map(evaluate,new.values())):
                evidence[identifier] = row
            rows = [evidence[key(job)] for job in jobs]
            for i,(sd,a) in enumerate(specs):
                group=rows[len(SEEDS)*i:len(SEEDS)*(i+1)]
                outcomes[sd][a]=all(r['held_in_place'] for r in group)
        run([(sd,a) for sd in ORIENTATIONS for a in grades[sd]])
        while True:
            specs=[]
            for sd in ORIENTATIONS:
                lo,hi=bounds(outcomes[sd])
                if hi is not None and hi-lo>1.01:
                    specs.append((sd,round((lo+hi)/2,2)))
            if not specs:
                break
            run(specs)
    summary=[]
    for sd in ORIENTATIONS:
        lo,hi=bounds(outcomes[sd])
        summary.append(dict(side=sd[0],orientation='uphill' if sd[1]<0 else 'downhill',
                            in_place_pass_deg=lo,in_place_miss_deg=hi))
    write(FOOTPRINT,dict(max_drift_mm=10.,summary=summary,trials=list(evidence.values())))
    print(summary,flush=True)


if __name__=='__main__':
    import argparse
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--study',type=Path,default=REPORT)
    args=parser.parse_args()
    qualify(args.workers,args.study)
