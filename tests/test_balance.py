import unittest

from reachy_locomotion.balance import BalanceController,BalanceFeedback,mirrored
from reachy_locomotion.primitives import Primitive


class BalanceFeedbackTests(unittest.TestCase):
    def test_rate_predicts_risk_before_angle_threshold(self):
        c=BalanceController(Primitive(),BalanceFeedback(trigger_deg=10,prediction_s=.1))
        c.target(.70,0);c.target(.72,4)
        self.assertIsNone(c.catch_trigger)
        c.target(.74,4)
        self.assertLess(c.sensed,10)
        self.assertGreater(c.predicted,10)
        self.assertEqual(c.catch_trigger,.74)

    def test_prediction_obeys_configured_sensor_delay(self):
        c=BalanceController(Primitive(),BalanceFeedback(sensor_delay_frames=2,trigger_deg=10))
        c.target(.70,0);c.target(.72,20);c.target(.74,20)
        self.assertIsNone(c.catch_trigger)
        c.target(.76,20)
        self.assertEqual(c.catch_trigger,.76)

    def test_mirror_changes_lateral_motion_but_preserves_forward_posture(self):
        p=Primitive(roll_deg=25,y_mm=12,yaw_deg=30,x_mm=10,pitch_deg=4)
        m=mirrored(p,-1)
        self.assertEqual((m.roll_deg,m.y_mm,m.yaw_deg),(-25,-12,-30))
        self.assertEqual((m.x_mm,m.pitch_deg),(10,4))

    def test_yaw_requires_delayed_roll_and_catch(self):
        c=BalanceController(Primitive(yaw_deg=30,yaw_at_roll_deg=5),
                            BalanceFeedback(trigger_deg=6,yaw_delay_after_catch_s=.2,prediction_s=0))
        c.target(.70,10);c.target(.72,10)
        self.assertEqual(c.catch_trigger,.72)
        self.assertIsNone(c.yaw_trigger)
        c.target(.94,10)
        self.assertEqual(c.yaw_trigger,.94)

    def test_signed_trigger_rejects_the_opposite_rocking_phase(self):
        c=BalanceController(Primitive(),BalanceFeedback(trigger_deg=8,prediction_s=0,catch_direction=-1))
        c.target(.70,12);c.target(.72,12)
        self.assertIsNone(c.catch_trigger)
        c.target(.74,-12);c.target(.76,-12)
        self.assertEqual(c.catch_trigger,.76)

    def test_head_brake_observation_is_delayed(self):
        c=BalanceController(Primitive(),BalanceFeedback(measured_yaw_brake=True))
        c.observe_head(0,0);c.observe_head(.02,20)
        self.assertEqual(c.head_yaw,0)
        c.observe_head(.04,20)
        self.assertEqual(c.head_yaw,20)
        self.assertGreater(c.head_rate,0)

    def test_recovery_override_is_delayed_and_latched(self):
        c=BalanceController(Primitive(),BalanceFeedback(emergency_roll_deg=20,emergency_prediction_s=0))
        c.target(.70,0);c.target(.72,25)
        self.assertIsNone(c.emergency_trigger)
        c.target(.74,25)
        self.assertEqual(c.emergency_trigger,.74)
        for frame in range(38,70):
            pose,stage,_=c.target(frame*.02,0)
        self.assertEqual(stage,'emergency recovery')
        self.assertEqual(c.emergency_trigger,.74)
        self.assertAlmostEqual(pose[2,3],.177)
        self.assertLess(abs(pose[1,3]),.011)

    def test_feedback_transition_waits_until_after_the_catch(self):
        c=BalanceController(Primitive(),BalanceFeedback(trigger_deg=2,prediction_s=0,roll_kp=-1.5,
                           recovery_gain_after_catch_s=.2,recovery_gain_blend_s=.12))
        for frame in range(36,45):c.target(frame*.02,4)
        self.assertLess(c.correction,0)
        for frame in range(45,75):c.target(frame*.02,4)
        self.assertAlmostEqual(c.correction,6,places=5)

    def test_timestamp_compensation_does_not_bypass_sensor_delay(self):
        c=BalanceController(Primitive(),BalanceFeedback(sensor_delay_frames=2,
                           sensor_latency_compensation=1,trigger_deg=10))
        c.target(.70,0);c.target(.72,20);c.target(.74,20)
        self.assertEqual(c.sensed,0)
        self.assertIsNone(c.catch_trigger)
        c.target(.76,20)
        self.assertEqual(c.sensed,20)
        self.assertEqual(c.catch_trigger,.76)


if __name__=='__main__':unittest.main()
