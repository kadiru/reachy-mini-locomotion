import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'))
from train_cadence import EdgeTriggeredSwing, cadence_measurement


class ActiveCadenceTests(unittest.TestCase):
    def test_steering_changes_only_on_completed_pairs(self):
        p = dict(swings_per_block=8, yaw=30., yaw_s=.1, yaw_delay=.05, brake=.08,
                 edge_gate=dict(delay_ticks=1,lead_s=0.,launch_deg=1.,max_wait_s=.02,
                                stop_deg=3.,min_fraction=.5),
                 pair_steering=dict(yaw_bias_per_deg=.01,delay_s_per_deg=.003))
        ctl = EdgeTriggeredSwing(p,1,8)
        for i in range(101):
            t = i/50
            ctl.observe_heading(t,15 if i>8 else 0)
            index = i//50
            ctl.yaw_axis(index,t-index,-1 if index%2==0 else 1,1 if index%2==0 else -1,0)
            if i==50:
                self.assertEqual(ctl.pair_heading,0)
        self.assertGreater(ctl.pair_heading,14)
        self.assertGreater(ctl.yaw_delay,.09)
    def test_encoder_brake_can_arrest_a_yaw_reference_after_its_ramp_ends(self):
        p = dict(swings_per_block=8, yaw=30., yaw_s=.1, yaw_delay=0., brake=.08,
                 edge_gate=dict(delay_ticks=1, lead_s=0., launch_deg=0., max_wait_s=0.,
                                stop_deg=10., min_fraction=.7, catch_deg=2., catch_range_deg=4.))
        ctl = EdgeTriggeredSwing(p,1,8)
        for tick in range(8):
            ctl.observe_head(tick*.02,20 if tick else 0)
            level = ctl.yaw_axis(0,tick*.02,0,1,3 if tick==6 else 0)
        self.assertAlmostEqual(level,26/30)
        self.assertEqual(ctl.stops,[1])
    def test_delayed_gyro_can_stop_a_stroke_and_next_stroke_starts_from_that_endpoint(self):
        policy = dict(swings_per_block=8, yaw_s=.2, yaw_delay=0., brake=.05,
                      edge_gate=dict(delay_ticks=1, lead_s=.1, launch_deg=.1, max_wait_s=0.,
                                     stop_deg=2., min_fraction=.25))
        ctl = EdgeTriggeredSwing(policy, 1, 8)
        for t in (0., .02, .04, .06):
            ctl.observe_angular_velocity(t, np.radians([50 if t == .06 else 0, 0, 0]))
            ctl.yaw_axis(0,t,0,1,0)
        self.assertEqual(ctl.stops, [])
        level = ctl.yaw_axis(0,.08,0,1,0)
        self.assertEqual(ctl.stops, [1])
        self.assertTrue(0 < level < 1)
        self.assertEqual(ctl.yaw_axis(0,.8,0,1,0),level)
        self.assertEqual(ctl.yaw_axis(1,0,1,-1,0),level)
    def test_compound_motion_counts_once_and_tail_cannot_dilute_cadence(self):
        t=np.arange(400)/50
        v=20*np.sin(np.pi*t)
        n,axes,_,_=cadence_measurement((v,v,v/2),8,8)
        self.assertEqual(n,max(8,*axes))
        resting=np.r_[v,np.repeat(v[-1],500)]
        self.assertEqual(cadence_measurement((resting,resting,resting/2),8,8)[0],n)

    def test_short_burst_hidden_by_average_is_detected(self):
        x=np.zeros(400)
        x[50:70]=20
        x[70:85]=-20
        x[85:99]=20
        x[99:120]=-20
        n,_,gap,window=cadence_measurement((x,x,0*x),8,8)
        self.assertEqual(n,8)
        self.assertLess(gap,.5)
        self.assertGreaterEqual(window,4)
