"""Predict neutral head tracking with a supported foot at a measured static roll angle."""
import argparse
import json

import mujoco
import numpy as np

from reachy_locomotion.contact_probe import relative_head_pose, scene_xml
from reachy_locomotion.geometry import angle, euler, rotation
from reachy_locomotion.motors import DynamixelApproximation
from tilted_release import MOTORS, UPSTREAM, plant, read


def fixed_fixture(roll):
    if not np.isfinite(roll) or abs(roll)>45:
        raise ValueError('Supported calibration uses signed roll up to 45 degrees')
    config=read(MOTORS)
    config.update(cad_shell_floor=True,timestep_s=.000125)
    free,targets=plant(True,config['timestep_s'],False)
    m=mujoco.MjModel.from_xml_string(scene_xml(UPSTREAM,config,free=False)[0])
    d=mujoco.MjData(m)
    # Use the foot frame. Tilting gravity is identical to rotating the rigid fixture.
    m.opt.gravity[:]=rotation(np.radians([roll,0.,0.])).T@np.array([0.,0.,-9.81])
    for j in range(m.njnt):
        source=free.model.joint(m.joint(j).name).id
        width=4 if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_BALL else 1
        dst,src=m.jnt_qposadr[j],free.model.jnt_qposadr[source]
        d.qpos[dst:dst+width]=free.initial_qpos[src:src+width]
    servo=DynamixelApproximation(m,config['motor_model'])
    servo.reset(targets)
    mujoco.mj_forward(m,d)
    return free,m,d,servo


def supported(roll):
    _,m,d,servo=fixed_fixture(roll)
    neutral=np.eye(4)
    neutral[2,3]=.177
    samples=[]
    for i in range(round(2./m.opt.timestep)):
        servo.apply(d)
        mujoco.mj_step(m,d)
        if i>=round(1.5/m.opt.timestep) and i%160==159:
            mujoco.mj_forward(m,d)
            head=relative_head_pose(m,d)
            samples.append([np.degrees(euler(head)[0]),1000*head[1,3],np.degrees(angle(neutral,head))])
    if any(w.number for w in d.warning) or not np.isfinite(samples).all():
        raise ValueError('Invalid supported model response')
    hr,y,err=np.median(samples,axis=0)
    return dict(supported_body_roll_deg=roll,neutral_target_head_roll_deg=0.,neutral_target_head_y_mm=0.,
                measured_head_roll_deg=float(hr),measured_head_y_mm=float(y),head_orientation_error_deg=float(err),
                settled_window_s=[1.5,2.],evidence='uncalibrated rigidly supported simulation')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('roll',type=float,nargs='+',help='Signed body roll, right positive')
    args=ap.parse_args()
    print(json.dumps([supported(a) for a in args.roll],indent=2,allow_nan=False))
