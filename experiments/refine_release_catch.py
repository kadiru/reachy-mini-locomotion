"""Fit release-dependent initial impulses and brakes; select specialists on development releases."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from train_release_catch import WORK, OLD, compare, ranked, read, write

PROBES=((30.3,-1.),(31.,1.8),(31.65,-.8),(32.2,0.),(32.85,-1.4),(33.45,1.4),(34.,.9))

def merit(r):
    b=r['balance']
    return (int(r['strict']),int(r['edge']), b['soft_hold_s'] - .3*(r['joint']<0),
            b['longest_strict_hold_s'],-min(r['tail_error'],70.))

def expand(pool):
    rng=np.random.default_rng(79107)
    seeds=read(WORK/'varied.json')
    out=[]
    for s in (1,-1):
        bases=[v['policy'] for v in ranked([q for q in seeds if q['side']==s and q['policy'].get('plane')])[:5]]
        candidates=[]
        for i in range(66):
            p=bases[i%len(bases)].copy()
            p['launch']=dict(duration=float(rng.choice([.04,.08,.12,.16,.20])),
                             mix=float(rng.uniform(.25,1.)),stroke=rng.uniform([-25,-20],[25,20]).tolist(),
                             brake=rng.uniform([-18,-12],[18,12]).tolist(),brake_s=float(rng.uniform(.08,.35)),
                             offset=rng.uniform([-2,-1.5],[2,1.5]).tolist(),damping=rng.uniform([-.9,-.7],[.9,.7]).tolist())
            p['lookahead']=float(rng.uniform(.05,.16))
            p['rate']=float(rng.choice([160,240,400,650,1000]))
            candidates.append((s,p))
        pairs=[(30.4,1.5,7521),(32.2,0.,4915),(33.75,-1.5,7522)]
        out+=compare(pool,candidates,pairs,1.4)
        write(WORK/'pulse_partial.json',out)
    return out

def select(pool):
    rows=read(WORK/'expand.json')
    old=read(WORK/'finalists.json')
    candidates=[]
    for side in (1,-1):
        chosen=[]
        for j in range(3):
            pool_rows=[r for r in rows if r['side']==side]
            for v in sorted(pool_rows,key=lambda v:merit(v['results'][j]),reverse=True)[:4]:
                if v['policy'] not in chosen: chosen.append(v['policy'])
        for v in ranked([x for x in old if x['side']==side])[:3]:
            if v['policy'] not in chosen: chosen.append(v['policy'])
        if OLD['left' if side<0 else 'right'] not in chosen:
            chosen.append(OLD['left' if side<0 else 'right'])
        candidates.extend((side,p) for p in chosen)
    return compare(pool,candidates,[(a,v,8201+i if i!=3 else 4915) for i,(a,v) in enumerate(PROBES)],5.8)

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage',choices=['expand','select'])
    ap.add_argument('--workers',type=int,default=6)
    args=ap.parse_args()
    with ProcessPoolExecutor(args.workers) as pool:
        write(WORK/(args.stage+'.json'),globals()[args.stage](pool))
