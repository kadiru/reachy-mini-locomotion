"""Bracket slow-angle and constant-speed limits on an initially level hinged deck."""
import argparse
import hashlib
import math
from concurrent.futures import ProcessPoolExecutor

from moving_table import MOTORS, POLICY, REPORT, SEEDS, SLOW, case, key, sweep, write
from pathlib import Path

ORIENTATIONS = [(side,direction) for side in (-1,1) for direction in (-1,1)]


def edge_pass(row):
    job=row['job']
    tracking=row['tracking_error_deg']<.1
    if job.get('cruise') and not job['flat']:
        tracking=tracking and abs(row['peak_table_rate']/job['rate']-1)<.03
    return bool(row['start_ok'] and row['continuous_edge'] and row['final_edge'] and tracking
                and row['termination']=='completed' and row['joint_margin']>=0 and row['shell'] is None)


def bounds(points):
    """Contiguous all-seed pass up to the first sampled miss. Preserve any later passes."""
    low=0.
    for value,passed in sorted(points.items()):
        if not passed:
            return low,value
        low=value
    return low,None


def archive(summary, trials):
    return {
        'schema_version': 1,
        'evidence': 'uncalibrated CAD-contact simulation',
        'policy_sha256': hashlib.sha256(POLICY.read_bytes()).hexdigest(),
        'motor_sha256': hashlib.sha256(Path(MOTORS).read_bytes()).hexdigest(),
        'seeds': list(SEEDS),
        'scores': {
            'edge_hold': 'See edge_pass in qualify_moving_table.py; used for the angle and rate brackets.',
            'passed': 'Same contact through all dwells, plus bounded pose drift in every endpoint window.',
            'quiet_threshold_pass': 'Also satisfies the earlier instantaneous base-speed and angular-rate limits.',
            'first_loss': 'Start of the first 20 consecutive milliseconds without valid support after motion begins.'
        },
        'summary': summary,
        'edge_hold_passes': sum(edge_pass(r) for r in trials),
        'trials': trials,
    }


def refine_rates(summary, workers):
    """Narrow the speed interval to 5% and pair its endpoints with level controls."""
    summary = [dict(group) for group in summary]
    evidence = {}
    with ProcessPoolExecutor(workers) as pool:
        def evaluate(jobs):
            rows = sweep(jobs, workers, pool)
            evidence.update({key(r['job']): r for r in rows})
            return rows

        while True:
            pending, jobs = [], []
            for group in summary:
                low, high = group['rate_pass'], group['rate_miss']
                if not low or high is None or high / low <= 1.05:
                    continue
                rate = round(math.sqrt(low * high), 2)
                side = group['side']
                sign = side if group['orientation']=='downhill' else -side
                pending.append((group, rate))
                jobs.extend(dict(case(side, sign*group['speed_amplitude'], rate, seed, 2), cruise=True)
                            for seed in SEEDS)
            if not pending:
                break
            rows = evaluate(jobs)
            for i, (group, rate) in enumerate(pending):
                repeats = rows[i*len(SEEDS):(i+1)*len(SEEDS)]
                group['rate_pass' if all(map(edge_pass,repeats)) else 'rate_miss'] = rate

        controls = []
        for group in summary:
            for rate in (group['rate_pass'], group['rate_miss']):
                if rate:
                    controls.extend(dict(case(group['side'], group['speed_amplitude'], rate, seed, 2, True),
                                         cruise=True) for seed in SEEDS)
        evaluate(controls)
    return summary, list(evidence.values())


def qualify(workers):
    evidence={}
    with ProcessPoolExecutor(workers) as pool:
        def run(jobs):
            rows=sweep(jobs,workers,pool)
            evidence.update({key(r['job']):r for r in rows})
            if jobs:
                print(f"{len(evidence)} unique trials, latest group {jobs[-1]}",flush=True)
            return rows

        def groups(specs):
            jobs=[]
            for (side,direction),amplitude,speed,cycles in specs:
                for seed in SEEDS:
                    job=case(side,side*direction*amplitude,speed,seed,cycles)
                    if cycles==2:
                        job['cruise']=True
                    jobs.append(job)
            rows=run(jobs)
            return [rows[i:i+len(SEEDS)] for i in range(0,len(rows),len(SEEDS))]

        slow={sd:{} for sd in ORIENTATIONS}
        rate={sd:{} for sd in ORIENTATIONS}
        baseline=groups([(sd,0.,SLOW,1) for sd in ORIENTATIONS])
        ready={sd:all(r['passed'] for r in rs) for sd,rs in zip(ORIENTATIONS,baseline)}
        active=[sd for sd in ORIENTATIONS if ready[sd]]
        for a in (2.,5.,10.,15.,20.,25.,30.,35.):
            if not active:break
            sets=groups([(sd,a,SLOW,1) for sd in active])
            for sd,rs in zip(active,sets):
                slow[sd][a]=all(edge_pass(r) for r in rs)
            if a>=10:
                active=[sd for sd in active if all(slow[sd].values())]

        def refine(tables,is_rate,amplitudes=None):
            while True:
                specs=[];levels=[]
                for sd,table in tables.items():
                    lo,hi=bounds(table)
                    if hi is None:continue
                    if is_rate:
                        if not lo or hi/lo<=1.42:continue
                        mid=round(math.sqrt(lo*hi),2)
                        specs.append((sd,amplitudes[sd],mid,2))
                    else:
                        if hi-lo<=1.01:continue
                        mid=round((lo+hi)/2,2)
                        specs.append((sd,mid,SLOW,1))
                    levels.append(mid)
                if not specs:break
                for spec,level,rs in zip(specs,levels,groups(specs)):
                    tables[spec[0]][level]=all(edge_pass(r) for r in rs)

        refine(slow,False)
        amplitudes={}
        # Prefer one shared amplitude and require two settled slow cycles on every tested edge.
        eligible=[sd for sd in ORIENTATIONS if bounds(slow[sd])[0]>0]
        for amplitude in (5.,2.,1.):
            candidates=[sd for sd in eligible if sd not in amplitudes and bounds(slow[sd])[0]>=amplitude]
            for sd,rs in zip(candidates,groups([(sd,amplitude,SLOW,2) for sd in candidates])):
                if all(r['passed'] and edge_pass(r) for r in rs):
                    amplitudes[sd]=amplitude
                    rate[sd][SLOW]=True
        active=list(amplitudes)
        for speed in (8.,32.,128.,256.):
            if not active:break
            for sd,rs in zip(active,groups([(sd,amplitudes[sd],speed,2) for sd in active])):
                rate[sd][speed]=all(edge_pass(r) for r in rs)
            active=[sd for sd in active if rate[sd][speed]]
        refine(rate,True,amplitudes)

        summary=[];controls=[]
        for sd in ORIENTATIONS:
            s,d=sd
            al,ah=bounds(slow[sd]);sl,sh=bounds(rate[sd])
            summary.append(dict(side=s,orientation='downhill' if d>0 else 'uphill',
                                baseline=ready[sd],angle_pass=al,angle_miss=ah,
                                speed_amplitude=amplitudes.get(sd),rate_pass=sl,rate_miss=sh))
            for a in (al,ah):
                if a:controls.extend(case(s,a,SLOW,n,1,True) for n in SEEDS)
            for v in (sl,sh):
                if v:controls.extend(dict(case(s,amplitudes[sd],v,n,2,True),cruise=True) for n in SEEDS)
        run(controls)
    summary, fine_rows = refine_rates(summary,workers)
    evidence.update({key(r['job']):r for r in fine_rows})
    write(REPORT,archive(summary,list(evidence.values())))
    print(summary,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workers',type=int,default=6)
    qualify(p.parse_args().workers)
