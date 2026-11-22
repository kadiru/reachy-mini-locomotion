"""Identify each CAD edge with the head pitch held neutral. Fit a delayed-input LQR."""
import argparse
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from reachy_locomotion.primitives import Primitive
from reachy_locomotion.release_plane import ReleasePlane
from tilted_release import plant,write
from train_release_catch import WORK


def sample(spec):
    side,seed=spec
    rng=np.random.default_rng(seed)
    sim,neutral=plant(True,.000125,False)
    sim.initial_targets=neutral.copy()
    policy=dict(interval=int(rng.choice([2,3,4,5,6])),amplitude=rng.uniform([3,2],[14,9]))
    ctl=ReleasePlane(side,seed,sim.checked_ik,policy,excitation=True)
    angle=float(rng.uniform(30.,34.))
    rate=float(rng.uniform(-3.,3.))
    run=sim.run_primitive(Primitive(),seconds=1.4,controller=ctl,waist_mode='hold_neutral',
        tilt_limit_deg=None,observe_landing=True,diagnostic_initial_roll_deg=side*angle,
        diagnostic_initial_angular_rate_deg_s=side*rate)
    xy=[]
    for a,b in zip(run['trace'],run['trace'][1:]):
        if a['time_s']<.06 or abs(a['plane_state'][0])>4.5 or abs(a['pitch_deg'])>4 or not a['edge_support'] or not b['edge_support']:
            continue
        xy.append((a['plane_state']+a['plane_action'],b['plane_state'][3:6]))
    return dict(side=side,seed=seed,angle=angle,outward_rate=rate,elapsed=run['elapsed_seconds'],xy=xy)


def identify(rows):
    x,y=zip(*(pair for row in rows for pair in row['xy']))
    x=np.c_[np.array(x),np.ones(len(x))];y=np.array(y)
    scale=np.array([2,7,5,20,60,35,8,6,8,6,8,6,1])
    z=x/scale
    weights=np.linalg.solve(z.T@z+.015*np.eye(13),z.T@y)/scale[:,None]
    A=np.zeros((10,10)); B=np.zeros((10,2)); c=np.zeros(10)
    A[3:6]=weights[:10].T; B[3:6]=weights[10:12].T;c[3:6]=weights[-1]
    A[:3]=.02*A[3:6]; A[:3,:3]+=np.eye(3); B[:3]=.02*B[3:6];c[:3]=.02*c[3:6]
    A[6:8,8:10]=np.eye(2); B[8:10]=np.eye(2)
    return dict(A=A.tolist(),B=B.tolist(),c=c.tolist(),samples=len(x),rate_rmse=np.sqrt(np.mean((x@weights-y)**2,axis=0)).tolist())


def design(model, angle_weight, trim, roll_cost=1.):
    A,B,c=[np.array(model[k]) for k in ('A','B','c')]
    Q=np.diag([angle_weight,.06,.12,.18,.002,.004,.001,.001,.001,.001])
    R=np.diag([roll_cost,2.]); P=Q.copy()
    for _ in range(1600):
        K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
        nxt=Q+A.T@P@(A-B@K)
        if np.max(abs(nxt-P))<1e-7:break
        P=(nxt+nxt.T)/2
    offset=np.zeros(10);offset[0]=trim
    H=np.zeros((10,4));H[1:3,:2]=np.eye(2);H[6:8,2:]=H[8:10,2:]=np.eye(2)
    U=np.c_[np.zeros((2,2)),np.eye(2)]
    raw=np.linalg.lstsq((np.eye(10)-A)@H-B@U,c-(np.eye(10)-A)@offset,rcond=None)[0]
    return dict(A=model['A'],B=model['B'],c=model['c'],K=K.tolist(),xeq=(offset+H@raw).tolist(),ueq=(U@raw).tolist())


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:
        rows=list(pool.map(sample,[(s,63100+i) for i in range(36) for s in (1,-1)]))
    models={str(s):identify([r for r in rows if r['side']==s]) for s in (-1,1)}
    write(WORK/'identification.json',dict(models=models,rollouts=[{k:v for k,v in r.items() if k!='xy'} for r in rows]))
