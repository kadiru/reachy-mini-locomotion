import importlib.util
import json
import unittest
from pathlib import Path

from reachy_locomotion.primitives import Primitive,PrimitiveController,classify_trial


class PrimitivePolicyTests(unittest.TestCase):
    def test_delayed_trigger_and_no_waist_command(self):
        p=Primitive(catch_at_deg=10);c=PrimitiveController(p)
        c.target(.7,0);c.target(.72,12)
        self.assertIsNone(c.catch_trigger)
        pose,stage,_=c.target(.74,12)
        self.assertEqual(c.catch_trigger,.74)
        self.assertEqual(stage,'catch')
        self.assertEqual(pose.shape,(4,4))

    def test_guard_stop_is_not_a_fall_or_recovery(self):
        tail=[dict(tilt_deg=0,base_angular_speed_rad_s=0,base_speed_m_s=0,normal_n=10)]
        r=classify_trial('actual_joint_limit_exceeded',tail,20)
        self.assertFalse(r['fell']);self.assertFalse(r['recovered']);self.assertFalse(r['model_valid'])
        self.assertTrue(classify_trial('nonfoot_floor_contact',tail,50)['fell'])
        self.assertTrue(classify_trial(None,tail,20)['recovered'])
        self.assertFalse(classify_trial(None,tail,2)['recovered'])


ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'experiments/upstream-reachy-mini'

@unittest.skipUnless(importlib.util.find_spec('mujoco') and (UPSTREAM/'.git').exists(),'Needs simulation assets')
class PrimitivePhysicsTests(unittest.TestCase):
    def test_disabled_waist_has_zero_torque_and_recording_does_not_change_physics(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        s=PrimitiveSimulator(UPSTREAM,json.loads((ROOT/'configs/motors-speed-trained.json').read_text()))
        p=Primitive(preload_roll_deg=-15,roll_deg=15,yaw_deg=15)
        a=s.run_primitive(p,seconds=1.2)
        b=s.run_primitive(p,seconds=1.2,record=True)
        self.assertEqual(a,{k:v for k,v in b.items() if k!='trajectory'})
        self.assertEqual(a['waist_peak_torque_nm'],0)
        self.assertEqual(a['waist_absolute_motor_work_j'],0)
        self.assertEqual(a['commanded_waist_position_range_rad'],[0.,0.])
        self.assertFalse(a['external_forces_applied'])
        self.assertTrue(a['model_valid'])
        self.assertTrue(all(x['waist_torque_nm']==0 for x in b['trajectory']))
        json.dumps(b, allow_nan=False)

    def test_fixed_waist_target_is_not_claimed_unpowered(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        s=PrimitiveSimulator(UPSTREAM,json.loads((ROOT/'configs/motors-speed-trained.json').read_text()))
        r=s.run_primitive(Primitive(preload_roll_deg=-15,roll_deg=15,yaw_deg=30),seconds=1.2,waist_mode='hold_neutral')
        self.assertEqual(r['commanded_waist_position_range_rad'],[0.,0.])
        self.assertGreater(r['waist_peak_torque_nm'],0)
        self.assertEqual(r['waist_mode'],'hold_neutral')
