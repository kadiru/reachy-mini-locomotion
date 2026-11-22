"""Choose launch and hold controllers on development data, then compare on reserved pairs."""
import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor

from train_release_catch import WORK, POLICY, OLD, RESERVED, read, write, batch, job
from refine_release_catch import PROBES

LEAD=.10
RETEST=((31.6,4913,.000125),(31.8,4914,.000125),(32.2,4915,.000125),
        (32.4,4916,.000125),(32.,4917,.0001),(32.,4918,.000125))


def grade(r):
    return (35*r['strict']+10*r['edge']+2*r['safe_flat']
            -4*(r['shell'] is not None)-3*(r['joint']<0)
            +r['balance']['longest_strict_hold_s']/2+r['balance']['soft_hold_s']/6)


def options(side):
    trials=[v for v in read(WORK/'select.json') if v['side']==side]
    filters=[v for v in read(WORK/'filters.json') if v['side']==side]
    central=sorted(filters,key=lambda v:grade(v['results'][0]),reverse=True)[:4]
    opts=[]
    for j in range(len(PROBES)):
        best=sorted(trials,key=lambda v:grade(v['results'][j]),reverse=True)[:2]
        selected=[v['policy'] for v in best]
        # Filter candidates hold the central edge quietly; test them on the adjacent bins.
        if j in (2,3,4):
            selected+=[v['policy'] for v in central[:4 if j==3 else 2]]
        old=OLD['left' if side<0 else 'right']
        if old not in selected:selected.append(old)
        opts.append(selected)
    return opts


def validate_candidates(pool):
    rows=[]
    centers=[a+v*LEAD for a,v in PROBES]
    for side in (1,-1):
        for j,policies in enumerate(options(side)):
            a,v=PROBES[j]
            conditions=[(a,v,4915 if j==3 else 8700+j,.000125),(a+.09,v-.3,8750+j,.000125)]
            for angle,seed,dt in RETEST:
                nearest=min(range(len(centers)),key=lambda k:abs(centers[k]-angle))
                condition=(angle,0.,seed,dt)
                if j==nearest and condition not in conditions:
                    conditions.append(condition)
            for p in policies:
                rows.append(dict(side=side,bin=j,policy=p,conditions=conditions))
    jobs=[job(r['side'],a,v,r['policy'],8.,seed,dt) for r in rows for a,v,seed,dt in r['conditions']]
    results=iter(batch(pool,jobs))
    for r in rows:
        r['results']=[next(results) for _ in r['conditions']]
    write(WORK/'validation.json',rows)


def freeze():
    data=read(WORK/'validation.json')
    policies={}
    for s in (-1,1):
        chosen=[]
        for i in range(len(PROBES)):
            candidates=[r for r in data if r['side']==s and r['bin']==i]
            def rank(r):
                grades=[grade(t) for t in r['results']]
                return sum(grades)+.5*min(grades)
            chosen.append(max(candidates,key=rank)['policy'])
        policies[str(s)]=dict(lead_s=LEAD,centers_deg=[a+v*LEAD for a,v in PROBES],specialists=chosen)
    write(POLICY,dict(evidence='uncalibrated CAD-contact simulation',reference_roll_deg=32.1,
                      control_interval_s=.02,waist_mode='hold_neutral',sides=policies))


def qualification(pool):
    policies=read(POLICY)['sides']
    jobs=[]
    labels=[]
    for s in (-1,1):
        old=OLD['left' if s<0 else 'right']
        for group,conditions in [('reserved',[(a,v,88801+i,.0001 if i%3==1 else .000125)
                                               for i,(a,v) in enumerate(RESERVED)]),
                                 ('retest',[(a,0.,seed,dt) for a,seed,dt in RETEST])]:
            for i,(a,v,seed,dt) in enumerate(conditions):
                comparisons=[('original',old),('trained',policies[str(s)])]
                if group=='reserved':
                    comparisons.append(('still',None))
                for name,p in comparisons:
                    clip=('trained-right' if s==1 else 'trained-left') if (
                        group=='retest' and i==2 and name=='trained') else None
                    if group=='reserved' and s==1 and name=='original':
                        clip=f'prior-right-{i}'
                    if group=='reserved' and name=='trained':
                        clip=f'fresh-{s}-{i}'
                    jobs.append(job(s,a,v,p,8.,seed,dt,clip))
                    labels.append(dict(group=group,controller=name,index=i,clip=clip))
    for s in (1,-1):
        for i,environment in enumerate([dict(reduced_bus=True),dict(sliding_friction=.55),
                                        dict(reduced_bus=True,sliding_friction=.55)]):
            for j,(angle,rate) in enumerate([(31.65,1.),(32.2,0.),(32.65,-1.)]):
                jobs.append(job(s,angle,rate,policies[str(s)],8.,94100+j)+ (environment,))
                labels.append(dict(group='transfer_sensitivity',controller='trained',index=3*i+j,clip=None))
    result=batch(pool,jobs)
    write(WORK/'qualification.json',dict(policy_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest(),
                                          trials=[dict(label,**r) for label,r in zip(labels,result)]))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage',choices=['validate','freeze','test'])
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    if a.stage=='freeze':
        freeze()
    else:
        with ProcessPoolExecutor(a.workers) as pool:
            (validate_candidates if a.stage=='validate' else qualification)(pool)
