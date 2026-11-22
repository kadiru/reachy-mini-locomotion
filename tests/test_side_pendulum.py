import math
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from reachy_locomotion.side_pendulum import add_pendulum


class PendulumTest(unittest.TestCase):
    def test_unlatched_bob_swings_inward_under_gravity(self):
        for side in (1,-1):
            root=ET.fromstring('<mujoco><option timestep="0.0005"/><worldbody/></mujoco>')
            add_pendulum(root,dict(side=side,release_deg=30.,anchor_xyz_m=[0,.3*side,.735],
                                   length_m=.62,radius_m=.025,mass_kg=.025))
            model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
            d=mujoco.MjData(model)
            for _ in range(1000):
                mujoco.mj_step(model,d)
            self.assertAlmostEqual(d.qpos[1],side*.61,places=3)
            self.assertLess(np.linalg.norm(d.qvel[:3]),.001)
            d.eq_active[model.equality('pendulum_latch').id]=False
            for _ in range(1200):
                mujoco.mj_step(model,d)
                if side*d.qpos[1]<.3:
                    break
            self.assertLess(side*d.qvel[1],0.)
            speed=math.sqrt(2*9.81*.62*(1-math.cos(math.radians(30.))))
            self.assertAlmostEqual(np.linalg.norm(d.qvel[:3]),speed,delta=.02)
            self.assertLess(abs(d.ten_length[0]-.62),.001)
