import math
import unittest

from reachy_locomotion.rocking import CycleMonitor, rocking_metrics
from reachy_locomotion.heading import HeadingController, HeadingFeedback, HeadingSensor


class RockingCycleTests(unittest.TestCase):
    def test_cycle_heading_allows_excursions_but_checks_every_endpoint(self):
        c = CycleMonitor(10,10,0.)
        for frame in range(31):
            phase=2*math.pi*frame/10
            c.observe(frame,[frame*.002,0,0],math.radians(10*math.sin(phase)),
                      math.radians(35*math.sin(phase)),[0,1] if math.sin(phase)>0 else [1,0])
        r=c.report()
        self.assertEqual(len(r['measured_cycles']),2)
        self.assertAlmostEqual(r['max_completed_cycle_heading_deg'],0)
        self.assertGreater(r['median_bilateral_roll_deg'],9)
        self.assertEqual(r['both_edges_cycle_fraction'],1)
        self.assertAlmostEqual(r['mean_forward_mm_per_cycle'],20)
        c.observe(40,[.08,0,0],0,math.radians(21),[1,0])
        self.assertGreater(c.report()['max_completed_cycle_heading_deg'],20)

    def test_signed_cycle_progress_does_not_reward_back_and_forth(self):
        c=CycleMonitor(10,0,0.)
        for i in range(11):
            c.observe(i,[math.sin(2*math.pi*i/10)*.1,0,0],0,0,[1,0])
        self.assertAlmostEqual(c.report()['mean_forward_mm_per_cycle'],0)

    def test_endpoint_controller_waits_for_delayed_same_phase_sample(self):
        c=HeadingController(HeadingFeedback(cycle_endpoint=True,heading_deadband_deg=0),
                            HeadingSensor(delay_frames=3,noise_std_deg=0),0.,cycle_frames=10)
        for frame in range(13):
            yaw=0 if frame==0 else (15 if frame==10 else 40)
            self.assertEqual(c.update(math.radians(yaw)),0)
        self.assertGreater(c.update(math.radians(40)),0)

    def test_style_constraints_reject_shuffle_and_falls(self):
        base=dict(valid=True,min_motor_joint_limit_margin_rad=.1,requested_cycles=1,measured_seconds=1,
                  measurement_displacement_in_startup_frame_m=[.02,0,0],max_heading_from_startup_rad=math.radians(40),
                  rocking_cycles=dict(measured_cycles=[{}],max_completed_cycle_heading_deg=10,
                                      mean_forward_mm_per_cycle=20,median_bilateral_roll_deg=8,both_edges_cycle_fraction=1))
        self.assertTrue(rocking_metrics(base)['feasible'])
        self.assertFalse(rocking_metrics(dict(base,valid=False))['stable'])
        self.assertFalse(rocking_metrics(dict(base,rocking_cycles=dict(base['rocking_cycles'],median_bilateral_roll_deg=2)))['style_ok'])
