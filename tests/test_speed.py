import math
import json
import unittest

import numpy as np
from reachy_locomotion.sway import Gait, brake_stroke
from experiments.train_speed import metrics
from test_sway import AVAILABLE, ROOT, UPSTREAM


class SpeedStrokeTests(unittest.TestCase):
    def test_stroke_is_continuous_periodic_bounded_with_finite_braking(self):
        for width in [.15, .5, 1.]:
            phase = np.linspace(-math.pi, math.pi, 4001)
            values = np.array([brake_stroke(p, width, .2, .04) for p in phase])
            self.assertLessEqual(abs(values).max(), 1.)
            for p in phase[::100]:
                self.assertAlmostEqual(brake_stroke(p, width, .2, .04), brake_stroke(p+2*math.pi, width, .2, .04), places=12)
            self.assertLess(np.max(np.abs(np.diff(values))), .03)
        # A short brake window reaches a genuine held endpoint without a jump.
        g = Gait(roll_deg=0, yaw_phase_deg=0, yaw_stroke_fraction=.5, stroke_brake_fraction=.025)
        np.testing.assert_allclose(g.pose(math.pi/2), g.pose(math.pi/2+.1), atol=1e-12)

    def test_speed_objective_rejects_fall_and_excess_heading_not_lateral_motion(self):
        r = dict(valid=True, measured_seconds=10., forward_m=.2,
                 measurement_displacement_in_startup_frame_m=[.2, .15, 0.],
                 max_heading_excursion_rad=.05, max_heading_from_startup_rad=math.radians(19),
                 min_motor_joint_limit_margin_rad=.1)
        self.assertTrue(metrics(r)['feasible'])
        fast = dict(r, measurement_displacement_in_startup_frame_m=[.4, .2, 0.])
        self.assertGreater(metrics(fast)['score'], metrics(r)['score'])
        self.assertFalse(metrics(dict(fast, valid=False))['feasible'])
        self.assertFalse(metrics(dict(fast, max_heading_from_startup_rad=math.radians(21)))['feasible'])
        self.assertFalse(metrics(dict(fast, min_motor_joint_limit_margin_rad=-.02))['stable'])

    def test_bad_profile_fractions_are_rejected(self):
        for fields in [dict(yaw_stroke_fraction=.01), dict(roll_stroke_fraction=1.1),
                       dict(stroke_brake_fraction=0), dict(stroke_accel_fraction=.5)]:
            with self.assertRaises(ValueError):
                Gait(**fields)

    def test_heading_deadband_preserves_unsteered_motion_inside_tolerance(self):
        from reachy_locomotion.heading import HeadingController, HeadingFeedback, HeadingSensor
        c = HeadingController(HeadingFeedback(heading_deadband_deg=10),
                              HeadingSensor(delay_frames=0, noise_std_deg=0), 0.)
        for _ in range(100):
            self.assertEqual(c.update(math.radians(8)), 0.)
        for _ in range(100):
            c.update(math.radians(15))
        self.assertGreater(c.correction_deg, 0.)
        self.assertLessEqual(c.correction_deg, c.config.max_correction_deg)


@unittest.skipUnless(AVAILABLE, 'Install simulator extras and pinned model')
class SpeedSimulationTests(unittest.TestCase):
    def test_fast_profile_keeps_physical_caps_and_distinguishes_rim_lift_from_flight(self):
        from reachy_locomotion.sway import SwaySimulator
        config = json.loads((ROOT/'configs/motors-datasheet.json').read_text())
        config['motor_model'].update(target_speed_rad_s=10.7, target_acceleration_rad_s2=360., target_filter_hz=25.)
        g = Gait(**json.loads((ROOT/'configs/sway-speed-trained.json').read_text()))
        sim = SwaySimulator(UPSTREAM, config)
        a = sim.run(g, cycles=3, warmup_cycles=3)
        b = sim.run(g, cycles=3, warmup_cycles=3, record=True)
        self.assertTrue(a['valid'], a['failure_reason'])
        self.assertEqual(a['forward_m'], b['forward_m'])
        for name, torque in zip(a['actuator_names'], a['peak_motor_torque_nm']):
            if name.startswith('stewart'):
                self.assertLessEqual(torque, .416+1e-10)
        self.assertLessEqual(a['motor_diagnostics']['max_filtered_command_speed_rad_s'], 10.7+1e-10)
        self.assertLessEqual(a['motor_diagnostics']['max_filtered_command_acceleration_rad_s2'], 360.+1e-9)
        self.assertGreater(a['max_side_rim_clearance_m'], .001)
        self.assertEqual(a['airborne_fraction'], 0.)
        self.assertEqual(a['longest_airborne_interval_s'], 0.)
        self.assertGreater(a['min_motor_joint_limit_margin_rad'], 0.)
        self.assertGreater(a['negative_motor_work_j'], 0.)
