import json
import math
import unittest
import numpy as np
from reachy_locomotion.heading import HeadingController, HeadingFeedback, HeadingSensor
from reachy_locomotion.sway import Gait, SwaySimulator
from test_sway import AVAILABLE, ROOT, UPSTREAM


class HeadingTests(unittest.TestCase):
    def test_wrap_delay_and_repeatable_sensor(self):
        sensor=HeadingSensor(delay_frames=3,noise_std_deg=0)
        ctrl=HeadingController(HeadingFeedback(),sensor,math.radians(179))
        for _ in range(3):
            ctrl.update(math.radians(-179))
            self.assertEqual(ctrl.filtered_deg,0)
        ctrl.update(math.radians(-179))
        self.assertAlmostEqual(ctrl.unwrapped_deg,2)
        self.assertGreater(ctrl.filtered_deg,0)
        a=HeadingController(HeadingFeedback(),HeadingSensor(seed=7),0)
        b=HeadingController(HeadingFeedback(),HeadingSensor(seed=7),0)
        self.assertEqual([a.update(.1) for _ in range(50)],[b.update(.1) for _ in range(50)])

    def test_correction_rate_envelope_and_antiwindup(self):
        c=HeadingController(HeadingFeedback(kp=1,ki=1,max_correction_deg=3,correction_rate_deg_s=5),
                            HeadingSensor(delay_frames=0,noise_std_deg=0),0)
        g=Gait(roll_deg=39,yaw_deg=59)
        last=0
        for _ in range(500):
            u=c.update(math.radians(90))
            self.assertLessEqual(abs(u-last),.1+1e-12);last=u
            self.assertLessEqual(abs(u),3)
            commanded=c.steer(g)
            self.assertLessEqual(abs(commanded.roll_deg)+abs(commanded.roll_bias_deg),40)
            self.assertLessEqual(abs(commanded.yaw_deg)+abs(commanded.yaw_bias_deg),60)
        self.assertLess(abs(c.integral),3)
        # A prolonged saturation must not prevent reversing the correction.
        for _ in range(150):c.update(math.radians(-10))
        self.assertLess(c.correction_deg,0)

    def test_reject_nonfinite_and_unbounded_settings(self):
        for kw in [dict(kp=float('nan')),dict(ki=-1),dict(max_correction_deg=20),dict(filter_tau_s=0)]:
            with self.assertRaises(ValueError):HeadingFeedback(**kw)
        for kw in [dict(delay_frames=-1),dict(noise_std_deg=-1),dict(bias_deg=float('inf'))]:
            with self.assertRaises(ValueError):HeadingSensor(**kw)

    def test_cycle_average_rejects_periodic_yaw_but_retains_drift(self):
        c=HeadingController(HeadingFeedback(cycle_average=True,filter_tau_s=.02,ki=0),
                            HeadingSensor(delay_frames=0,noise_std_deg=0),0,cycle_frames=27)
        for frame in range(270):c.update(.1*math.sin(2*math.pi*frame/27))
        self.assertLess(abs(c.filtered_deg),1e-10)
        for frame in range(270,540):c.update(math.radians(5)+.1*math.sin(2*math.pi*frame/27))
        self.assertAlmostEqual(c.filtered_deg,5,places=9)
        self.assertGreater(c.correction_deg,0)

    def test_excursion_adaptation_reduces_and_recovers_stroke(self):
        c=HeadingController(HeadingFeedback(yaw_excursion_target_deg=3,min_yaw_scale=.5),
                            HeadingSensor(delay_frames=0,noise_std_deg=0),0)
        for frame in range(270):c.update(math.radians(10)*math.sin(2*math.pi*frame/27))
        self.assertAlmostEqual(c.yaw_scale,.5)
        for frame in range(270,810):c.update(math.radians(1)*math.sin(2*math.pi*frame/27))
        self.assertAlmostEqual(c.yaw_scale,1.)
        self.assertEqual(c.minimum_yaw_scale,.5)

    def test_harmonic_steering_preserves_shared_yaw_envelope(self):
        c=HeadingController(HeadingFeedback(kp=2,roll_per_correction=0,yaw_harmonic_per_correction=-2,
                            yaw_harmonic_phase_deg=90,max_correction_deg=6),
                            HeadingSensor(delay_frames=0,noise_std_deg=0),0)
        for _ in range(200):c.update(math.radians(10))
        g=c.steer(Gait(yaw_deg=58,yaw_bias_deg=2))
        self.assertLessEqual(abs(g.yaw_deg)+abs(g.yaw_bias_deg)+abs(g.yaw_harmonic_deg),60)
        self.assertLess(g.yaw_harmonic_deg,0)
        self.assertEqual(g.roll_bias_deg,0)

    def test_phase_estimator_separates_drift_and_compensates_delay(self):
        c=HeadingController(HeadingFeedback(phase_fit=True,filter_tau_s=.02),
                            HeadingSensor(delay_frames=5,noise_std_deg=0),0)
        for frame in range(400):
            heading=frame*.02*.5+8*math.sin(2*math.pi*frame/27)+2*math.cos(4*math.pi*frame/27)
            # Initial measurement includes the periodic component's initial offset.
            c.update(math.radians(heading))
        self.assertAlmostEqual(c.filtered_deg,399*.02*.5,delta=.02)
        self.assertAlmostEqual(c.fit_rate,.5,places=8)


@unittest.skipUnless(AVAILABLE,'Install sim extras and fetch pinned model')
class HeadingSimulationTests(unittest.TestCase):
    def test_feedback_recording_and_motor_caps(self):
        sim=SwaySimulator(UPSTREAM,json.loads((ROOT/'configs/motors-datasheet.json').read_text()))
        g=Gait(**json.loads((ROOT/'configs/sway-progress-fast.json').read_text()))
        feedback=dict(kp=.08,ki=.04)
        a=sim.run(g,cycles=4,feedback=feedback)
        b=sim.run(g,cycles=4,feedback=feedback,record=True)
        self.assertTrue(a['valid'],a['failure_reason'])
        self.assertEqual(a['forward_m'],b['forward_m'])
        self.assertEqual(a['max_heading_from_startup_rad'],b['max_heading_from_startup_rad'])
        self.assertFalse(b['external_forces_applied'])
        self.assertGreater(b['heading_feedback']['peak_correction_deg'],0)
        self.assertLessEqual(max(b['peak_motor_torque_nm'][1:7]),.416+1e-10)
        # The only command to the body-yaw motor remains zero.
        body=sim.act[0]
        self.assertEqual(sim.servos.target[body],0)
        for row in b['trajectory']:
            self.assertLessEqual(abs(row['target_head_rpy_rad'][0]),math.radians(40)+1e-10)
            self.assertTrue(np.isfinite(row['heading_estimate_deg']))
