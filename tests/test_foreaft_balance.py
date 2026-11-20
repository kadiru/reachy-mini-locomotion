import unittest
import numpy as np

from reachy_locomotion.foreaft_balance import ForeAftController,foreaft_metrics,physical_pose,virtual_pose
from reachy_locomotion.geometry import rotation,euler
from reachy_locomotion.edge_balance import EdgeFeedback
from reachy_locomotion.primitives import Primitive


class ForeAftTests(unittest.TestCase):
    def test_coordinate_conversion_round_trips_a_general_pose(self):
        p=np.eye(4);p[:3,:3]=rotation([.25,-.18,.12]);p[:3,3]=[.01,-.02,.177]
        np.testing.assert_allclose(physical_pose(virtual_pose(p)),p,atol=1e-15)

    def test_positive_virtual_roll_commands_forward_pitch(self):
        p=np.eye(4);p[:3,:3]=rotation(np.radians([30,0,0]));p[:3,3]=[.01,.02,.177]
        q=physical_pose(p);np.testing.assert_allclose(np.degrees(euler(q)),[0,30,0],atol=1e-12)
        np.testing.assert_allclose(q[:3,3],[-.02,.01,.177])

    def test_controller_transforms_physical_pitch_before_delayed_feedback(self):
        c=ForeAftController(Primitive(),EdgeFeedback(trigger_deg=25,prediction_s=0),1,lambda p:np.zeros(7))
        c.observe_attitude(.8,30);c.target(.8,0);self.assertIsNone(c.catch_trigger)
        c.observe_attitude(.82,30);c.target(.82,0);self.assertEqual(c.catch_trigger,.82)

    def test_physical_angle_envelope_is_checked_after_rotation(self):
        c=ForeAftController(Primitive(),EdgeFeedback(),1,lambda p:np.zeros(7))
        p=np.eye(4);p[:3,:3]=rotation(np.radians([40,40,0]));p[2,3]=.177
        with self.assertRaises(ValueError):c.checked_virtual_ik(p)

    @staticmethod
    def result(side):
        force=[14.,0.] if side==-1 else [0.,14.]
        return dict(completed=True,trace=[dict(time_s=.02*i,pitch_deg=side*32.1,pitch_rate_deg_s=0.,roll_deg=0.,
            foreaft_edge_support=True,foot_front_back_normal_forces_n_back_front=force,front_back_height_difference_mm=59.) for i in range(301)])

    def test_both_requested_edges_hold_for_six_seconds(self):
        for side in [-1,1]:
            m=foreaft_metrics(self.result(side),side,32.1)
            self.assertEqual(m['longest_strict_hold_s'],6);self.assertTrue(m['final_five_seconds_pass'])

    def test_wrong_contact_axis_or_transient_motion_cannot_pass(self):
        for change in [dict(foreaft_edge_support=False),dict(foot_front_back_normal_forces_n_back_front=[14,0]),
                       dict(pitch_deg=0),dict(pitch_rate_deg_s=6),dict(roll_deg=6)]:
            r=self.result(1)
            for x in r['trace']:x.update(change)
            self.assertFalse(foreaft_metrics(r,1,32.1)['final_five_seconds_pass'])

    def test_terminated_or_short_trial_cannot_pass(self):
        r=self.result(1);r['completed']=False;self.assertFalse(foreaft_metrics(r,1,32.1)['final_five_seconds_pass'])
        r=self.result(1);r['trace']=r['trace'][:250];self.assertFalse(foreaft_metrics(r,1,32.1)['final_five_seconds_pass'])


if __name__=='__main__':unittest.main()
