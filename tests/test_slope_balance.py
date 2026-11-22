import importlib.util
import json
import sys
import unittest
from pathlib import Path

import numpy as np

from reachy_locomotion.contact_probe import validate_config

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'))


class RampConfigurationTests(unittest.TestCase):
    def test_invalid_bank_is_rejected(self):
        config=json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        for angle in (True,90,-90,float('nan')):
            with self.subTest(angle=angle),self.assertRaises(ValueError):
                validate_config(dict(config,floor_roll_deg=angle))


@unittest.skipUnless(importlib.util.find_spec('mujoco') and (ROOT/'experiments/upstream-reachy-mini/.git').exists(),'Needs CAD assets')
class RampPhysicsTests(unittest.TestCase):
    def test_measured_start_bank_selects_the_requested_uphill_or_downhill_stroke(self):
        from slope_balance import RampPump
        from types import SimpleNamespace
        common=dict(actions=[[0.,0.,0.]],frames=4,lead=0.,trigger=90.,
                    min_trigger=90.,bank_gain=-.045,strength=1.)
        choices=[dict(relative_bank=b,settings=dict(common,reference_bank=b)) for b in (-5.,5.)]
        sim=SimpleNamespace(checked_ik=lambda pose: None)
        for bank in (-6.,6.):
            for side in (-1,1):
                pump=RampPump(sim,side,dict(bins=choices),84)
                for i in range(10):
                    pump.target(i*.02,bank)
                pump.target(.2,0.)
                self.assertAlmostEqual(pump.bank,bank,delta=.1)
                self.assertEqual(pump.selected['reference_bank'],np.sign(bank*side)*5)
                self.assertEqual(pump.strength<1.,bank*side>0)

    def test_flat_start_is_measured_relative_to_both_ramps(self):
        from slope_balance import plant
        from reachy_locomotion.primitives import Primitive
        for bank in (-9.,9.):
            with self.subTest(bank=bank):
                sim=plant(bank,.000125,.7)
                np.testing.assert_allclose(sim.model.opt.gravity,[0,0,-9.81])
                result=sim.run_primitive(Primitive(),seconds=1.,waist_mode='hold_neutral',record=True,
                                         tilt_limit_deg=None,observe_landing=True)
                row=result['trace'][0]
                self.assertAlmostEqual(row['roll_deg'],bank,delta=.1)
                self.assertLess(row['tilt_from_floor_deg'],.1)
                self.assertLess(row['side_height_difference_mm'],.15)
                self.assertFalse(row['edge_support'])
                self.assertGreater(min(result['trajectory'][0]['foot_side_normal_forces_n_right_left']),.5)

    def test_rigid_pivot_barrier_matches_height_gain(self):
        from slope_barrier import barrier
        import math
        p=np.array([[0.,0.]])
        com=np.array([.05,.1])
        for bank in (-6.,0.,6.):
            r=barrier(p,com,1.5,bank,1,.05)
            b=math.radians(bank)
            zmax=math.hypot(*com)
            expected=zmax-com[0]*math.sin(b)-com[1]*math.cos(b)
            self.assertAlmostEqual(r['com_rise_mm'],1000*expected,places=3)
