import unittest

import numpy as np

from reachy_locomotion.continuous_spin import Orbit, SpinController, spin_metrics


class SpinTests(unittest.TestCase):
    def test_head_orbit_repeats_and_can_hold_a_fixed_pose(self):
        orbit = Orbit(12, 15., 12., -90., 10., 15., 5., 7., -4.)
        controller = SpinController(orbit, lambda pose: np.zeros(6),
                                    np.tile([-1.,1.],(6,1)), hold_at=1.30)
        pose = controller.target(1.,0.)[0]
        np.testing.assert_array_equal(pose, controller.target(1.24,0.)[0])
        last = controller.target(1.28,0.)[0]
        np.testing.assert_array_equal(last, controller.target(1.90,0.)[0])

    def test_mirroring_preserves_the_workspace_margin(self):
        orbit = Orbit(12, 25., 15., -90., 20., 0., 4., 0., 0.)
        def lateral_joint(pose):
            return np.array([20 * pose[1, 3]])
        limits = np.array([[-.25, .25]])
        left = SpinController(orbit, lateral_joint, limits, mirror=1)
        right = SpinController(orbit, lateral_joint, limits, mirror=-1, direction=-1)
        reflection = np.diag([1., -1., 1., 1.])
        for frame in range(60, 72):
            pose = left.target(frame / 50, 0.)[0]
            mirrored = right.target(frame / 50, 0.)[0]
            self.assertLessEqual(abs(lateral_joint(pose)[0]), .25 - .08)
            np.testing.assert_allclose(mirrored, reflection @ pose @ reflection, atol=1e-12)
        # This point requires workspace projection along the requested trajectory.
        self.assertGreater(left.target(1.2, 0.)[0][1, 3], .007)

    def test_counts_complete_body_turns_and_rejects_an_oscillating_base(self):
        for direction in (1,-1):
            trace = [dict(time_s=t,foot_yaw_deg=direction*18*t,torso_yaw_deg=direction*18*t,
                          foot_xyz_m=[0.,0.,0.],roll_deg=8.,pitch_deg=0.,edge_support=True,
                          foreaft_edge_support=False,normal_n=10.) for t in np.arange(3001)*.02]
            result = dict(trace=trace,completed=True,peak_tilt_deg=8.,min_joint_margin_rad=.1,
                          commanded_waist_position_range_rad=[0.,0.],termination='completed',fell=False)
            m = spin_metrics(result,60.,direction)
            self.assertTrue(m['passed'])
            self.assertAlmostEqual(m['rpm'],3.)
            self.assertAlmostEqual(m['rim_base_deg'],1080.*direction)
            for row in trace:
                row['foot_yaw_deg']=20*np.sin(2*np.pi*row['time_s'])
            self.assertFalse(spin_metrics(result,60.,direction)['passed'])
