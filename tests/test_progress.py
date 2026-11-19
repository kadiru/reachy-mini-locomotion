import json
import math
import unittest

import numpy as np
from reachy_locomotion.geometry import euler
from reachy_locomotion.sway import Gait,SwaySimulator
from test_sway import AVAILABLE,ROOT,UPSTREAM


class StrokeTests(unittest.TestCase):
    def test_bounded_periodic_dwell_and_startup(self):
        g=Gait(roll_deg=35,yaw_deg=50,yaw_phase_deg=0,roll_sharpness=2,yaw_sharpness=3)
        for p in np.linspace(0,2*math.pi,151):
            np.testing.assert_allclose(g.pose(p),g.pose(p+2*math.pi),atol=1e-12)
            r,pitch,y=euler(g.pose(p))
            self.assertLessEqual(abs(r),math.radians(35)+1e-12)
            self.assertLessEqual(abs(y),math.radians(50)+1e-12)
        np.testing.assert_allclose(g.pose(.8,0)[:3,:3],np.eye(3),atol=1e-12)
        self.assertGreater(euler(g.pose(math.pi/3))[2],math.radians(49))
        # The hold has a flat finite endpoint, without discontinuous position.
        self.assertLess(np.max(np.abs(g.pose(math.pi/2+1e-5)-g.pose(math.pi/2-1e-5))),1e-9)

    def test_bias_and_amplitude_share_angle_envelope(self):
        for kw in [dict(roll_deg=39,roll_bias_deg=2),dict(yaw_deg=55,yaw_bias_deg=6),dict(yaw_sharpness=6)]:
            with self.assertRaises(ValueError):Gait(**kw)

    def test_constraint_rejects_spinning_that_returns_to_start(self):
        from experiments.train_progress import metrics
        r=dict(valid=True,measured_seconds=6.,forward_m=.1,max_lateral_excursion_m=0.,
               max_heading_excursion_rad=2*math.pi,heading_change_rad=0.,max_foot_tilt_deg=5.)
        self.assertFalse(metrics(r)['feasible'])
        r['max_heading_excursion_rad']=.05
        self.assertTrue(metrics(r)['feasible'])


@unittest.skipUnless(AVAILABLE,'Install sim extras and fetch pinned model')
class StrokeSimulationTests(unittest.TestCase):
    def test_recording_does_not_change_physics_and_retains_contact_momentum(self):
        sim=SwaySimulator(UPSTREAM,json.loads((ROOT/'configs/motors-datasheet.json').read_text()))
        g=Gait(roll_deg=30,yaw_deg=30,yaw_sharpness=1,frequency_hz=2)
        a=sim.run(g,cycles=3);b=sim.run(g,cycles=3,record=True)
        self.assertTrue(a['valid'],a['failure_reason'])
        self.assertEqual(a['forward_m'],b['forward_m'])
        self.assertEqual(a['max_heading_excursion_rad'],b['max_heading_excursion_rad'])
        self.assertGreaterEqual(a['max_heading_excursion_rad'],abs(a['unwrapped_heading_change_rad'])-1e-12)
        for row in b['trajectory']:
            self.assertTrue(np.isfinite(row['head_subtree_angular_momentum_world']).all())
            self.assertTrue(np.all(np.array(row['foot_side_normal_forces_n_right_left'])>=0))
        self.assertLessEqual(max(b['peak_motor_torque_nm'][1:7]),.416+1e-10)
