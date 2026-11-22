"""Estimate a lower balance angle by holding the head toward the intended edge."""
import math
import mujoco
import numpy as np
from reachy_locomotion.contact_probe import relative_head_pose
from reachy_locomotion.geometry import euler,rotation
from slope_balance import action_pose
from slope_barrier import geometry
from supported_tilt_model import fixed_fixture


def fit(spec):
    side,roll,y=spec
    free,m,d,servo=fixed_fixture(side*29.)
    vert,_,_=geometry()
    base=m.body('body_foot_3dprint').id
    for scale in (1.,.9,.8,.65,.5):
        action=[roll*scale,y*scale,0.]
        try:
            target=free.checked_ik(action_pose(side,action))
            break
        except ValueError:
            if scale==.5:raise
    full=free.initial_targets.copy();full[free.act]=target
    angle=side*29.;seconds=0.
    for _ in range(3):
        m.opt.gravity[:]=rotation(np.radians([angle,0.,0.])).T@[0.,0.,-9.81]
        for tick in range(50):
            servo.command(full)
            for _ in range(round(.02/m.opt.timestep)):
                servo.apply(d);mujoco.mj_step(m,d)
        seconds+=1.
        mujoco.mj_forward(m,d)
        world=d.xmat[base].reshape(3,3)
        com=(world.T@(d.subtree_com[base]-d.xpos[base]))[1:]
        # The side rim has the same local support facet across the training inclines.
        rad=math.radians(angle)
        contact=vert[np.argmin(vert[:,0]*math.sin(rad)+vert[:,1]*math.cos(rad))]
        angle=math.degrees(math.atan2(com[0]-contact[0],com[1]-contact[1]))
    head=relative_head_pose(m,d)
    return dict(side=side,action=action[:2],equilibrium=angle*side,head_roll=float(side*np.degrees(euler(head)[0])),
                head_y=float(side*head[1,3]*1000),seconds=seconds)
