import copy
import json
import unittest
from pathlib import Path

import numpy as np
from reachy_locomotion.motors import dc_torque, specification, validate_motor_config
from test_sway import AVAILABLE, ROOT, UPSTREAM


class MotorMathTests(unittest.TestCase):
    def test_manufacturer_endpoints_and_no_extrapolation(self):
        torque, speed=specification('XL330-M288',5.)
        self.assertAlmostEqual(torque,.52)
        self.assertAlmostEqual(speed,103*2*np.pi/60)
        for v in [3.,6.4,float('nan')]:
            with self.assertRaises(ValueError):specification('XL330-M288',v)

    def test_torque_speed_relation_and_braking(self):
        self.assertAlmostEqual(float(dc_torque(1,0,.52,10,.52)),.52)
        self.assertAlmostEqual(float(dc_torque(1,10,.52,10,.52)),0.)
        self.assertLess(float(dc_torque(1,11,.52,10,.52)),0.)
        self.assertAlmostEqual(float(dc_torque(-1,10,.52,10,.416)),-.416)

    def test_invalid_derating_and_latency_are_rejected(self):
        c=json.loads((ROOT/'configs/motors-datasheet.json').read_text())['motor_model']
        for key,value in [('torque_derating',1.2),('delay_frames',.5),('target_speed_rad_s',0),('passive_friction_nm',-.01)]:
            bad=dict(c);bad[key]=value
            with self.assertRaises(ValueError):validate_motor_config(bad)


@unittest.skipUnless(AVAILABLE,'Install sim extras and fetch pinned model')
class MotorSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from reachy_locomotion.sway import SwaySimulator
        cls.config=json.loads((ROOT/'configs/motors-datasheet.json').read_text())
        cls.sim=SwaySimulator(UPSTREAM,cls.config)

    def test_mass_joint_limits_and_passive_joint_inertia(self):
        m=self.sim.model
        self.assertAlmostEqual(float(m.body_subtreemass[self.sim.base]),1.475)
        self.assertAlmostEqual(self.sim.command_limits[1,0],np.radians(-48))
        for j in range(m.njnt):
            if m.joint(j).name.startswith('passive'):
                adr=m.jnt_dofadr[j]
                np.testing.assert_array_equal(m.dof_armature[adr:adr+3],0)

    def test_rollout_obeys_caps_and_repeats_without_external_force(self):
        from reachy_locomotion.sway import Gait
        g=Gait(frequency_hz=3,roll_deg=30,yaw_deg=20)
        a=self.sim.run(g,cycles=3);b=self.sim.run(g,cycles=3)
        self.assertTrue(a['valid'],a['failure_reason'])
        self.assertEqual(a['forward_m'],b['forward_m'])
        self.assertFalse(a['external_forces_applied'])
        d=a['motor_diagnostics']
        self.assertTrue(np.all(np.array(a['peak_motor_torque_nm'])<=np.array(d['torque_caps_nm'])+1e-9))
        self.assertLessEqual(d['max_filtered_command_speed_rad_s'],6.+1e-9)
        self.assertLessEqual(d['max_filtered_command_acceleration_rad_s2'],60.+1e-9)

    def test_zero_motion_baseline(self):
        from reachy_locomotion.sway import Gait
        r=self.sim.run(Gait(roll_deg=0,yaw_deg=0),cycles=4)
        self.assertTrue(r['valid'])
        self.assertLess(np.hypot(r['forward_m'],r['lateral_m']),1e-5)
