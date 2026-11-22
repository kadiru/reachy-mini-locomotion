"""Tune complementary attitude and command filters against uninterrupted strict holds."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import numpy as np

from train_release_catch import WORK, compare, read, write
from refine_release_catch import merit
from identify_release_trim import identify


def filters(pool):
    rng=np.random.default_rng(78102)
    passive=identify(pool)
    write(WORK/'neutral-trim.json',passive)
    previous=read(WORK/'finalists.json')
    candidates=[]
    for side in (1,-1):
        best=sorted([v for v in previous if v['side']==side and v['policy'].get('plane')],
                    key=lambda v:merit(v['results'][2]),reverse=True)[:3]
        for i in range(64):
            p=best[i%3]['policy'].copy()
            if i%2:
                p['neutral_trim']=True
                p['model']=dict(p['model'],xeq=passive['trims'][str(side)]['state'],ueq=[0.,0.])
            p['filter']=dict(attitude_s=float(rng.uniform(.045,.45)),rate_s=float(rng.uniform(.005,.055)),
                             command_s=float(rng.uniform(.004,.075)))
            p['gain']*=float(rng.uniform(.65,1.15))
            p['rate']=float(rng.uniform(65,260))
            candidates.append((side,p))
    return compare(pool,candidates,[(32.2,0.,4915)],3.4)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:
        write(WORK/'filters.json',filters(pool))
