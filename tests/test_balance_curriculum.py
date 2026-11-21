import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np

from reachy_locomotion.balance_curriculum import BalanceReturn


class SensedController:
    def __init__(self):
        self.side=1;self.sensed=np.zeros(2);self.rate=np.zeros(2)
        self.f=SimpleNamespace(target_roll_deg=32.1)
        self.command_history=deque([np.zeros(4),np.zeros(4)],maxlen=2)

    def target(self,t,roll):
        self.sensed[0]=roll;pose=np.eye(4);pose[2,3]=.177
        return pose,'hold',roll

    def diagnostics(self):return {}


class BalanceReturnTests(unittest.TestCase):
    def test_emergency_requires_prior_stable_balance(self):
        c=SensedController();r=BalanceReturn(c,hold_s=30,roll_deg=20,y_mm=-15,duration_s=.2,
                                            emergency_roll_deg=33,emergency_rate_deg_s=8)
        c.rate[0]=20;r.target(0,34);self.assertIsNone(r.return_start)
        c.rate[0]=0;r.target(.1,32.1);r.target(.7,32.1)
        c.rate[0]=10;r.target(.72,33.5)
        self.assertEqual(r.return_start,.72);self.assertTrue(r.emergency_triggered)

    def test_normal_return_requires_continuous_hold_and_tracks_actual_command(self):
        c=SensedController();r=BalanceReturn(c,hold_s=1,roll_deg=20,y_mm=-15,duration_s=.4)
        r.target(0,32.1);r.target(.8,28);r.target(1,32.1);r.target(1.8,32.1)
        self.assertIsNone(r.return_start)
        r.target(2.1,32.1);pose,stage,_=r.target(2.5,31)
        self.assertEqual(stage,'controlled return');self.assertAlmostEqual(pose[1,3],-.015)
        np.testing.assert_allclose(c.command_history[-1],[20,0,0,-15],atol=1e-10)
        r.target(2.6,15);self.assertEqual(r.neutral_start,2.6)
        pose,_,_=r.target(5,0);np.testing.assert_allclose(pose[:3,3],[0,0,.177]);self.assertTrue(r.finished)


if __name__=='__main__':unittest.main()
