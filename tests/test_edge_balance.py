import copy
import unittest

import numpy as np

from reachy_locomotion.edge_balance import EdgeFeedback,EdgeBalanceController,balance_metrics
from reachy_locomotion.primitives import Primitive


class EdgeBalanceTests(unittest.TestCase):
    @staticmethod
    def result(seconds=6,side=1):
        return dict(completed=True,trace=[dict(time_s=i*.02,roll_deg=side*30,roll_rate_deg_s=0,pitch_deg=0,
                    edge_support=True,com_world_m=[0,0,.1],contact_world_m=[0,0,0]) for i in range(round(seconds/.02)+1)])

    def test_stationary_supported_hold_passes_both_sides(self):
        for side in [-1,1]:
            m=balance_metrics(self.result(side=side),side,30)
            self.assertTrue(m['final_five_seconds_pass']);self.assertEqual(m['longest_strict_hold_s'],6)

    def test_flat_wrong_edge_transient_and_terminated_trials_fail(self):
        r=self.result()
        variants=[]
        flat=copy.deepcopy(r)
        for x in flat['trace']:x['roll_deg']=0
        variants.append(flat)
        wrong=copy.deepcopy(r)
        for x in wrong['trace']:x['roll_deg']=-30
        variants.append(wrong)
        fast=copy.deepcopy(r)
        for x in fast['trace']:x['roll_rate_deg_s']=30
        variants.append(fast)
        aborted=copy.deepcopy(r);aborted['completed']=False;variants.append(aborted)
        variants.append(self.result(seconds=4.98))
        for v in variants:self.assertFalse(balance_metrics(v,1,30)['final_five_seconds_pass'])

    def test_loss_of_support_breaks_continuous_hold(self):
        r=self.result();r['trace'][150]['edge_support']=False
        m=balance_metrics(r,1,30)
        self.assertFalse(m['final_five_seconds_pass']);self.assertEqual(m['longest_strict_hold_s'],2.98)

    def test_single_sample_has_zero_hold_duration(self):
        self.assertEqual(balance_metrics(self.result(seconds=0),1,30)['longest_strict_hold_s'],0)

    def test_capture_obeys_side_and_sensor_delay(self):
        c=EdgeBalanceController(Primitive(),EdgeFeedback(trigger_deg=20,prediction_s=0,sensor_delay_frames=2),-1,lambda p:np.zeros(7))
        for t in [.70,.72,.74]:c.target(t,25)
        self.assertIsNone(c.catch_trigger)
        c.target(.76,-25);c.target(.78,-25);self.assertIsNone(c.catch_trigger)
        c.target(.80,-25);self.assertEqual(c.catch_trigger,.80)

    def test_projection_checks_every_returned_pose(self):
        def ik(pose):
            if abs(pose[1,3])>.002:raise ValueError('test workspace')
            return np.zeros(7)
        c=EdgeBalanceController(Primitive(preload_y_mm=25),EdgeFeedback(),1,ik)
        for i in range(50):
            pose,_,_=c.target(i*.02,0);ik(pose)
        self.assertGreater(c.projected_frames,0)

    def test_no_delayed_rate_impulse_after_artificial_stationary_reset(self):
        c=EdgeBalanceController(Primitive(),EdgeFeedback(),1,lambda p:np.zeros(7));c.hold_from_start=True
        c.target(0,32);c.target(.02,32)
        self.assertEqual(c.rate[0],0)

    def test_invalid_inputs_rejected(self):
        for kwargs in [dict(target_roll_deg=40),dict(sensor_delay_frames=-1),dict(roll_kd_s=float('nan')),dict(y_bias_mm=30)]:
            with self.assertRaises(ValueError):EdgeFeedback(**kwargs)


if __name__=='__main__':unittest.main()
