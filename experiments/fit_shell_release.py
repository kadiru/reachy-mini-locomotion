"""Identify local roll/pitch response directly on the CAD foot, then fit a regulator."""
import argparse
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from fit_edge_regulator import fit, regulator
from reachy_locomotion.edge_linear import LinearEdgeController
from tilted_release import EDGES, WORK, plant, read, write


def excite(job):
    side, seed = job
    sim, neutral = plant(True,.000125,False)
    sim.initial_targets = neutral.copy()
    p = read(EDGES)['edges']['left' if side < 0 else 'right']
    feedback = dict(p['feedback'],sensor_delay_frames=0,sensor_noise_deg=0.)
    ctl = LinearEdgeController(p['recipe'],feedback,side,sim.checked_ik,excitation_seed=seed)
    ctl.hold_from_start = True
    angle = 32.1+np.random.default_rng(seed+17).uniform(-.7,.7)
    r = sim.run_primitive(p['recipe'],seconds=1.2,controller=ctl,waist_mode='hold_neutral',
                          diagnostic_initial_roll_deg=side*angle,observe_landing=True,tilt_limit_deg=None)
    return dict(trace=[{k:row[k] for k in ('time_s','edge_state','edge_action')} for row in r['trace']])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=5)
    a=parser.parse_args()
    jobs=[(s,79023+i) for i in range(24) for s in (-1,1)]
    with ProcessPoolExecutor(a.workers) as pool:
        rows=list(pool.map(excite,jobs))
    models={}
    for s in (-1,1):
        data=[row for (side,_),row in zip(jobs,rows) if side==s]
        A,B,c,xeq,ueq,stats=fit(data)
        K,radius=regulator(A,B,angle_weight=50,action_weight=1.)
        models[str(s)]=dict(A=A.tolist(),B=B.tolist(),c=c.tolist(),x_equilibrium=xeq.tolist(),
                            u_equilibrium=ueq.tolist(),gain=K.tolist(),fit=stats,spectral_radius=radius)
    write(WORK/'cad-model.json',models)


if __name__=='__main__':
    main()
