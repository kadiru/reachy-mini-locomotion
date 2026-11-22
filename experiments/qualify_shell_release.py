"""Select on the CAD development runs and evaluate neighboring stationary releases."""
import argparse
from concurrent.futures import ProcessPoolExecutor

from tilted_release import WORK, read, run, write


def settings(rows, side):
    candidates=[r for r in rows if r['job']['side']==side and r['job']['angle']==32 and r['job']['mode']=='feedback']
    def rank(r):
        b=r['balance']
        return (r['edge_pass'],r['min_joint_margin_rad']>=0 and not r['first_body_floor_contact'],
                b['longest_strict_hold_s'],-b['tail_max_roll_rate_deg_s'])
    return max(candidates,key=rank)['job']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workers',type=int,default=5)
    a=p.parse_args()
    development=sum((read(WORK/(f+'.json')) for f in ('tilt','fit','local','facet')),[])
    winners=[settings(development,s) for s in (-1,1)]
    jobs=[]
    for j in winners:
        jobs.append(dict(j,seconds=20.,record=f"long-{j['side']}"))
        for angle,seed,dt in ((31.6,4913,.000125),(31.8,4914,.000125),(32.2,4915,.000125),
                              (32.4,4916,.000125),(32,4917,.0001),(32,4918,.000125)):
            jobs.append(dict(j,seconds=8.,angle=angle,seed=seed,dt=dt))
    with ProcessPoolExecutor(a.workers) as pool:
        results=list(pool.map(run,jobs))
    write(WORK/'qualification.json',dict(winners=winners,results=results))


if __name__=='__main__':
    main()
