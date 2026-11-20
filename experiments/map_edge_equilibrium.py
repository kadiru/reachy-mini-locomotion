"""Anchored diagnostic COM map, NOT a demonstration of free-foot balance."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from reachy_locomotion.contact_probe import scene_xml
from reachy_locomotion.geometry import rotation
from reachy_locomotion.motors import DynamixelApproximation
from reachy_locomotion.primitives import PrimitiveSimulator, blend


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    config=json.loads(Path('configs/motors-speed-trained.json').read_text());config['timestep_s']=.0005
    sim=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    xml,_=scene_xml('experiments/upstream-reachy-mini',config,free=False)
    model=mujoco.MjModel.from_xml_string(xml);servos=DynamixelApproximation(model,config['motor_model'])
    base=model.body('body_foot_3dprint').id;rows=[]
    for roll in [-35,0,35]:
        for y in [-20,0,20]:
            data=mujoco.MjData(model)
            for j in range(model.njnt):
                old=sim.model.joint(model.joint(j).name).id;n=4 if model.jnt_type[j]==mujoco.mjtJoint.mjJNT_BALL else 1
                data.qpos[model.jnt_qposadr[j]:model.jnt_qposadr[j]+n]=sim.initial_qpos[sim.model.jnt_qposadr[old]:sim.model.jnt_qposadr[old]+n]
            servos.reset(sim.initial_targets);valid=True
            for frame in range(101):
                u=blend(frame*.02);pose=np.eye(4);pose[:3,:3]=rotation(np.radians([u*roll,0,0]));pose[:3,3]=[0,u*y/1000,.177]
                try:target=sim.checked_ik(pose)
                except ValueError:valid=False;break
                targets=sim.initial_targets.copy();targets[sim.act]=target;servos.command(targets)
                for _ in range(40):servos.apply(data);mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data);com=data.subtree_com[base]-data.xpos[base]
            row=dict(head_roll_deg=roll,head_y_mm=y,valid=valid,com_local_m=com.tolist(),
                     positive_roll_equilibrium_deg=math.degrees(math.atan2(.056+com[1],com[2])))
            rows.append(row);print(row,flush=True)
    out.write_text(json.dumps(dict(evidence='uncalibrated_simulation',anchored_diagnostic_only=True,rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
