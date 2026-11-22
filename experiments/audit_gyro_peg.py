"""Audit material-point rolling at every solver step and benchmark the original pivot."""
import argparse
import math

import mujoco
import numpy as np

from reachy_locomotion.landing_pivot import LandingPivot
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.rim_roll import measures
from train_gyro_peg import MODELS, POLICY, WORK, controller_for, simulator, trial
from train_target_pivot import read,write


class StepContact:
    def __init__(self):
        self.rows = []
        self.wrench = np.empty(6)

    def __call__(self, sim, d):
        if d.time < 1.5: return
        rot = d.xmat[sim.base].reshape(3,3)
        base = d.xpos[sim.base]
        force=0.
        weighted=np.zeros(3)
        for i,c in enumerate(d.contact):
            if {c.geom1,c.geom2}!={sim.foot,sim.floor} or c.efc_address<0:continue
            mujoco.mj_contactForce(sim.model,d,i,self.wrench)
            normal=abs(self.wrench[0])
            if normal>.01:
                force+=normal
                weighted+=normal*c.pos
        local=world=None
        slip=0.
        if force:
            point=weighted/force
            local=(rot.T@(point-base))[:2].tolist()
            world=point.tolist()
            jac=np.empty((3,sim.model.nv))
            mujoco.mj_jac(sim.model,d,jac,None,point,sim.base)
            slip=float(np.linalg.norm((jac@d.qvel)[:2]))
        self.rows.append(dict(time_s=d.time,normal_n=force,contact_foot_xy_m=local,contact_world_m=world,
                              tilt_deg=math.degrees(math.acos(np.clip(rot[2,2],-1.,1.))),
                              contact_slip_m_s=slip,foot_xyz_m=base.tolist()))


def audit(policy,condition,mirror,seconds=24.):
    sim=simulator(condition)
    sign=policy['foot_winding_for_positive_mirror']*mirror
    ctl=controller_for(sim,policy['orbit'],condition,mirror)
    sensor=StepContact()
    run=sim.run_primitive(Primitive(),seconds=seconds,controller=ctl,waist_mode='hold_neutral',
                          tilt_limit_deg=None,step_audit=sensor,record_rim_contact=True)
    radius=run['config']['foot_radius_m']
    return dict(condition=condition,mirror=mirror,hz=1/sim.dt,samples=len(sensor.rows),
                termination=run['termination'], fell=run['fell'],
                control_frames=measures(run['trace'],radius,sign),integration_steps=measures(sensor.rows,radius,sign))


def pivot_reference():
    from train_slick_quarter_turn import CONFIG,OPTIONS
    sim=simulator('check_seed')
    delay,noise,seed=MODELS['check_seed'][-3:]
    ctl=LandingPivot(OPTIONS,-90,sim.checked_ik,guidance=read(CONFIG),seed=seed,delay=delay,noise=noise)
    run=sim.run_primitive(Primitive(),seconds=4.4*ctl.max_strokes+1.,controller=ctl,waist_mode='hold_neutral',
                          tilt_limit_deg=None,finish_settle_s=.8,record_rim_contact=True)
    return dict(condition='check_seed',target=-90., termination=run['termination'],torso_yaw_deg=run['final_torso_yaw_deg'],
                strokes=len(ctl.strokes), **measures(run['trace'],run['config']['foot_radius_m']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    a=p.parse_args()
    policy=read(POLICY)
    from fit_gyro_course import WIDE,choice
    checks=read(WORK/'course-checks.json')
    featured=max([r for r in checks if r['condition']!='nominal'],key=choice)
    winner=read(WIDE)
    step=StepContact()
    replay=trial((winner['orbit'],featured['condition'],featured['mirror'],featured['seconds'],
                  featured['direction'],WORK/'wide-best.json'),course=True,step_audit=step)
    if replay!=featured:
        raise ValueError('Featured held-out trial changed during high-rate contact audit')
    radius=simulator(featured['condition']).config['foot_radius_m']
    dense=measures(step.rows,radius,featured['direction'])
    del step
    write(a.output,dict(pivot_reference=pivot_reference(),
                        audits=[audit(policy,'nominal',m) for m in (1,-1)],
                        wide_audit=dict(condition=featured['condition'],mirror=featured['mirror'],
                                        hz=1/MODELS[featured['condition']][5],integration_steps=dense)))
