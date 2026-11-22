"""Tune the braking handoff, freeze inclination bins, and test the identical starts."""
import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from slope_balance import POLICY, WORK, grade, read, run, spec, write

BANKS=(0.,5.,-5.,9.,-9.)
FINAL_BANKS=(*BANKS,2.5,-2.5)
UNSEEN=(-8.,-6.5,-4.,-2.,2.,4.,6.5,8.)


def fit_brakes(pool):
    rng=np.random.default_rng(533814)
    plans=read(WORK/'plans.json')
    validated=read(WORK/'validate.json')
    all_rows=[]
    for bank in BANKS:
        jobs=[]
        for side in (-1,1):
            arm=next(v for v in plans if v['bank']==bank and v['side']==side)
            matched=[v for v in validated if v['result']['bank']==bank and v['result']['side']==side]
            options=[v['settings'] for v in arm['nearest'][:4]]
            options+=[v['settings'] for v in sorted(matched,key=lambda v:grade(v['result']),reverse=True)[:2]]
            for i in range(54):
                p=options[i%len(options)].copy()
                if i>=len(options):
                    p.update(min_trigger=float(rng.uniform(15.,29.)),trigger=float(rng.uniform(24.,34.)),
                             lead=float(rng.uniform(.01,.18)),inertia=float(rng.uniform(-.1,1.4)),
                             max_rate=float(rng.choice([22.,40.,80.,250.])),warm=bool(i%3),
                             strength=float(rng.uniform(.89,1.10)),catch_model=str(rng.choice(['plane','full'])))
                    if 'actions' in p:
                        p['unlock']=max(.2,p['unlock']+float(rng.uniform(-.24,.08)))
                        p['coast']=True
                jobs.append(spec(bank,side,p,seconds=3.8))
        all_rows.extend(dict(settings=j[2],result=r) for j,r in zip(jobs,pool.map(run,jobs)))
        write(WORK/'brakes-progress.json',all_rows)
    return all_rows


def shortlist(rows,bank,side,n):
    matched=[v for v in rows if v['result']['bank']==bank and v['result']['side']==side]
    options=[]
    for v in sorted(matched,key=lambda v:(v['result'].get('hold',False),grade(v['result'])),reverse=True):
        if v['settings'] not in options:options.append(v['settings'])
        if len(options)>=n:break
    return options


def freeze(pool):
    from steep_slope_balance import train as train_steep
    rows=read(WORK/'brakes.json')+read(WORK/'coast.json')+read(WORK/'validate.json')
    rows.extend(train_steep(pool))
    out=[];sides={};compensation=[]
    for s in (-1,1):
        bins=[]
        for b in (*FINAL_BANKS,23.*s):
            here=[v['result'] for v in rows if v['result']['bank']==b and v['result']['side']==s]
            # Allocate more repeats where the launch has already found a quiet edge.
            promising=any(r['edge'] or r.get('longest_quiet_s',0.)>=.12 for r in here)
            opts=shortlist(rows,b,s,4 if promising else 2)
            jobs=[spec(b+delta,s,p,seconds=8. if delta==0. else 6.,seed=seed) for p in opts
                  for delta,seed in ((0.,55001),(.18,60231),(-.18,60341))]
            observed=list(pool.map(run,jobs))
            records=[dict(settings=p,results=observed[3*i:3*i+3]) for i,p in enumerate(opts)]
            def rank(row):
                v=[grade(r) for r in row['results']]
                rs=row['results']
                return (sum(r['hold'] for r in rs),sum(r['edge'] for r in rs),
                        -sum(r['fell'] or r['joint']<0 for r in rs),sum(v)+.4*min(v))
            out.extend(records)
            winner=max(records,key=rank)
            selected=winner['settings']
            measured_reference=s*winner['results'][0]['sensed_bank']
            variants=[dict(selected,reference_bank=measured_reference,bank_gain=k) for k in (0.,-.045,.045)]
            jobs=[spec(b+delta,s,p,seconds=6.,seed=seed) for p in variants for delta,seed in ((-.65,70519),(.65,70629))]
            trials=list(pool.map(run,jobs))
            comparisons=[dict(settings=p,results=trials[2*i:2*i+2]) for i,p in enumerate(variants)]
            compensation.extend(comparisons)
            bins.append(dict(relative_bank=s*b,settings=max(comparisons,key=rank)['settings']))
        sides[str(s)]=dict(bins=sorted(bins,key=lambda x:x['relative_bank']))
    write(WORK/'selection.json',out)
    write(WORK/'compensation.json',compensation)
    write(POLICY,dict(evidence='uncalibrated CAD-contact simulation',waist='hold neutral',sides=sides))


def qualify(pool):
    policy=read(POLICY)
    jobs=[];labels=[]
    for suite,banks in [('matched',FINAL_BANKS),('unseen',UNSEEN),('steep',(-23.,23.,-22.5,22.5,-23.5,23.5))]:
        for i,b in enumerate(banks):
            for s in (-1,1):
                learned=policy['sides'][str(s)]
                level=dict(next(x['settings'] for x in learned['bins'] if x['relative_bank']==0),bank_gain=0.)
                for name,p in [('original',None),('level_only',level),('adaptive',learned)]:
                    seed=55001 if suite=='matched' else 90300+i
                    clip=f'{suite}-{i}-{s}' if name=='adaptive' else None
                    jobs.append(spec(b,s,p,8.,seed,clip=clip)+(True,))
                    labels.append(dict(suite=suite,controller=name,clip=clip))
    for b in (-9.,-5.,5.,9.):
        for s in (-1,1):
            for dt,mu in ((.0001,.7),(.000125,.60)):
                jobs.append(spec(b,s,policy['sides'][str(s)],8.,97123,dt,mu)+(True,))
                labels.append(dict(suite='sensitivity',controller='adaptive',clip=None))
    for s in (-1,1):
        for seed in (105031,105117,105241,105371):
            jobs.append(spec(9.*s,s,policy['sides'][str(s)],8.,seed)+(True,))
            labels.append(dict(suite='nine_degree_repeat',controller='adaptive',clip=None))
    result=[dict(label,**r) for label,r in zip(labels,pool.map(run,jobs))]
    write(WORK/'qualification.json',dict(policy_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest(),trials=result))


def strength(pool):
    policy=read(POLICY)['sides']
    jobs=[];labels=[]
    for b in (-5.,5.):
        for s in (-1,1):
            bins=policy[str(s)]['bins']
            for family,reference in [('selected',s*b),('common_level_stroke',0.)]:
                base=next(v['settings'] for v in bins if v['relative_bank']==reference)
                for k in (.55,.7,.85,.95,1.,1.05,1.15):
                    p=dict(base,strength=k*base.get('strength',1.),bank_gain=0.)
                    jobs.append(spec(b,s,p,8.,55001)+(True,))
                    labels.append(dict(family=family,scale=k))
    return [dict(label,**r) for label,r in zip(labels,pool.map(run,jobs))]


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task',choices=['brakes','freeze','qualify','strength'])
    a=p.parse_args()
    with ProcessPoolExecutor(6) as pool:
        if a.task=='brakes':write(WORK/'brakes.json',fit_brakes(pool))
        elif a.task=='freeze':freeze(pool)
        elif a.task=='qualify':qualify(pool)
        else:write(WORK/'strength.json',strength(pool))
