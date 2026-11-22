"""Measure how the original controller responds to restricting individual head actions."""
from concurrent.futures import ProcessPoolExecutor
import math
import numpy as np

from reachy_locomotion.geometry import euler, rotation
from reachy_locomotion.primitives import Primitive
from tilted_release import balance_controller, plant, write
from train_release_catch import WORK, OLD
from reachy_locomotion.edge_balance import balance_metrics


class RestrictHead:
    def __init__(self, child, sim, setting):
        self.child, self.sim, self.setting = child, sim, setting
        self.projected = 0

    def __getattr__(self, name):
        return getattr(self.child,name)

    def target(self,t,roll):
        pose,stage,sensed = self.child.target(t,roll)
        if self.setting == 'original':
            return pose,stage,sensed
        angles = euler(pose[:3,:3])
        xyz = pose[:3,3].copy()
        if self.setting in ('quiet fore-aft','both'):
            xyz[0] = 0.
            angles[1] = 0.
        if self.setting in ('cap roll','both'):
            angles[0] = np.clip(angles[0], -math.radians(6), math.radians(6))
        for scale in (1.,.85,.65,.4,0.):
            trial = np.eye(4)
            trial[:3,:3] = rotation(scale*angles)
            trial[:3,3] = [scale*xyz[0],scale*xyz[1],.177+scale*(xyz[2]-.177)]
            try:
                self.sim.checked_ik(trial)
                break
            except ValueError:
                if scale == 0.:
                    raise
        self.projected += scale < 1.
        return trial,stage,sensed


def trial(spec):
    s,setting=spec
    sim,neutral=plant(True,.000125,False)
    sim.initial_targets=neutral.copy()
    old=balance_controller(sim,s,dict(OLD['right'],regulator_side=1,seed=4915))
    ctl=RestrictHead(old,sim,setting)
    r=sim.run_primitive(Primitive(),seconds=8.,controller=ctl,waist_mode='hold_neutral',tilt_limit_deg=None,
                        observe_landing=True,record=True,diagnostic_initial_roll_deg=s*32.2)
    b=balance_metrics(r,s,32.1)
    return dict(side=s,setting=setting,first_shell=r['first_body_floor_contact'],settled=r['settled'],
                joint_margin_rad=r['min_joint_margin_rad'],balance=b,projected_frames=ctl.projected,
                head_roll_at_half_second_deg=float(np.degrees(r['trajectory'][25]['head_rpy_rad'][0])),
                mirrored_roll_at_half_second_deg=s*r['trace'][25]['roll_deg'])


if __name__=='__main__':
    with ProcessPoolExecutor(2) as pool:
        rows=list(pool.map(trial,[(s,k) for s in (1,-1) for k in ('original','quiet fore-aft','cap roll','both')]))
    write(WORK/'ablation.json',rows)
