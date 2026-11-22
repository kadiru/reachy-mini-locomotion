"""Bracket the impulse that coasts towards the tipping point, then close the catch feedback."""
from concurrent.futures import ProcessPoolExecutor
from slope_balance import WORK, spec, run, read, write, grade


def fit(pool):
    from slope_head_trim import fit as fit_trim
    trims=list(pool.map(fit_trim,[(s,a,y) for s in (-1,1) for a,y in [(18.,-12.),(30.,-19.)]]))
    write(WORK/'trims.json',trims)
    from plan_slope_strokes import plan
    extra=list(pool.map(plan,[(b,s,True) for b in (-2.5,2.5,-5.,5.,-9.,9.,0.) for s in (-1,1)
                              if abs(b)==2.5 or b*s<0 or (b==0 and s==-1)]))
    write(WORK/'extra-plans.json',extra)
    plans=read(WORK/'plans.json')
    by_pair={(v['bank'],v['side']):v for v in plans}
    for arm in extra:
        key=(arm['bank'],arm['side'])
        if key in by_pair:
            existing=by_pair[key]
            existing['nearest']=sorted(existing['nearest']+arm['nearest'],key=lambda v:v['cost'])
        else:
            by_pair[key]=arm
    plans=list(by_pair.values())
    table=read(WORK/'validate.json')
    searches=[];jobs=[];conditions=[]
    for tree in plans:
        b,s=tree['bank'],tree['side']
        matched=[r for r in table if r['result']['bank']==b and r['result']['side']==s and r['result']['trigger'] is not None]
        sources=[(v['settings'],.20+len(v['settings']['actions'])*v['settings']['frames']*.02) for v in tree['nearest'][:3]]
        sources += [(v['settings'],v['result']['trigger']) for v in sorted(matched,key=lambda r:grade(r['result']),reverse=True)[:2]]
        if not sources:
            # Use a successful slope neighbor as a pump when the first search could not reach this edge.
            other=min((u for u in plans if u['side']==s and u['nearest']),key=lambda u:abs(u['bank']-b))
            for v in other['nearest'][:2]:
                p=v['settings'];sources.append((p,.20+len(p['actions'])*p['frames']*.02))
        for launch,stop in sources:
            for offset,trim in [(-.08,None),(.04,None),(-.08,next(x for x in trims if x['side']==s)),
                                (0.,[x for x in trims if x['side']==s][-1])]:

                p=dict(launch,stop_s=max(.28,stop+offset),trigger=10000.,min_trigger=1000.)
                if trim:p['trim']=trim
                ids=[]
                for scale in (.78,.92,1.,1.08,1.20):
                    ids.append(len(jobs));jobs.append(spec(b,s,dict(p,strength=scale),seconds=max(3.4,p['stop_s']+1.6)))
                conditions.append((b,s,p,ids))
    coarse=list(pool.map(run,jobs))
    brackets=[]
    for b,s,p,ids in conditions:
        samples=[(jobs[i][2]['strength'],coarse[i]) for i in ids]
        for i in range(len(samples)-1):
            lo,hi=samples[i:i+2]
            if lo[1]['outward_escape']!=hi[1]['outward_escape']:
                brackets.append(dict(bank=b,side=s,settings=p,lo=lo,hi=hi,
                                     quality=min(lo[1]['coast_cost'],hi[1]['coast_cost'])))
        searches.append(dict(bank=b,side=s,settings=p,samples=samples))
    active=[]
    for b,s in set((v['bank'],v['side']) for v in brackets):
        active+=sorted([v for v in brackets if (v['bank'],v['side'])==(b,s)],key=lambda v:v['quality'])[:2]
    refine=[]
    for _ in range(7):
        midjobs=[]
        for v in active:
            scale=(v['lo'][0]+v['hi'][0])/2
            p=dict(v['settings'],strength=scale)
            midjobs.append(spec(v['bank'],v['side'],p,seconds=max(3.4,p['stop_s']+1.6)))
        for v,j,r in zip(active,midjobs,pool.map(run,midjobs)):
            scale=j[2]['strength']
            branch='lo' if r['outward_escape']==v['lo'][1]['outward_escape'] else 'hi'
            v[branch]=(scale,r)
            refine.append(dict(settings=j[2],result=r))
    candidates=[dict(settings=dict(v['settings'],strength=scale),result=r) for v in searches for scale,r in v['samples']]+refine
    write(WORK/'coast-search.json',candidates)
    evaljobs=[]
    for tree in plans:
        b,s=tree['bank'],tree['side']
        group=sorted([v for v in candidates if v['result']['bank']==b and v['result']['side']==s],
                     key=lambda v:v['result']['coast_cost'])
        for v in group[:4]:
            target=v['settings'].get('trim',{}).get('equilibrium',31.55)
            for gap,rate,wait in ((3.5,30.,.08),(1.5,18.,.1),(.75,9.,.14),(2.5,70.,.04)):
                a=target-gap
                p=dict(v['settings'],min_trigger=a,trigger=a,lead=.035,max_rate=rate,
                       unlock=v['settings']['stop_s']+wait,catch_model='full' if s<0 and rate==18. else 'plane',warm=True)
                evaljobs.append(spec(b,s,p,seconds=6.))
                if gap==1.5 or gap==3.5:
                    evaljobs.append(spec(b,s,dict(p,pitch_damping=.65),seconds=6.))
    return [dict(settings=j[2],result=r) for j,r in zip(evaljobs,pool.map(run,evaljobs))]


if __name__=='__main__':
    with ProcessPoolExecutor(6) as pool:
        write(WORK/'coast.json',fit(pool))
