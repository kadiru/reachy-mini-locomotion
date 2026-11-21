import unittest
import numpy as np
from reachy_locomotion.edge_linear import LinearEdgeController
from reachy_locomotion.edge_balance import EdgeFeedback
from reachy_locomotion.primitives import Primitive


def model():
    return dict(A=np.eye(20).tolist(),B=np.zeros((20,4)).tolist(),c=np.zeros(20).tolist(),
                gain=np.zeros((4,20)).tolist(),x_equilibrium=np.zeros(20).tolist(),u_equilibrium=[0,0,0,0])


class LinearEdgeTests(unittest.TestCase):
    def make(self,side=1,delay=1,linear=None):
        return LinearEdgeController(Primitive(),EdgeFeedback(sensor_delay_frames=delay),side,lambda p:np.zeros(7),linear or model())

    def test_head_pose_observation_uses_relative_pose_and_mirrors_only_lateral_axes(self):
        c=self.make(-1);pose=np.eye(4);pose[:3,3]=[.003,.004,.177]
        c.observe_head_state(0,pose);c.target(0,-32.1)
        self.assertEqual(c.observed[4],3);self.assertEqual(c.observed[5],-4)

    def test_delayed_observation_does_not_see_new_angle(self):
        c=self.make();c.target(0,32.1);c.target(.02,33.1)
        self.assertAlmostEqual(c.state[0],0);c.target(.04,33.1)
        self.assertAlmostEqual(c.state[0],1)

    def test_command_prediction_uses_own_history(self):
        m=model();m['B'][0][0]=.1
        c=self.make(linear=m);c.command_history[-1][0]=4;c.target(0,32.1)
        self.assertAlmostEqual(c.state[0],.4)

    def test_bad_fitted_model_rejected(self):
        for key,value in [('gain',[[0]]),('A',np.full((20,20),np.nan).tolist())]:
            m=model();m[key]=value
            with self.assertRaises(ValueError):self.make(linear=m)

    def test_simulated_gyro_has_the_same_delay_as_attitude(self):
        c=self.make();c.configure_sensors(dict(gyro_std_deg_s=0))
        c.observe_angular_velocity(0,[0,0,0]);c.target(0,32.1)
        c.observe_angular_velocity(.02,np.radians([10,0,0]));c.target(.02,32.1)
        self.assertEqual(c.state[6],0)
        c.observe_angular_velocity(.04,np.radians([10,0,0]));c.target(.04,32.1)
        self.assertAlmostEqual(c.state[6],10)

    def test_noisy_angles_do_not_get_differentiated_when_gyro_is_used(self):
        c=self.make(delay=0);c.configure_sensors(dict(gyro_std_deg_s=0))
        for i,a in enumerate([32.1,32.2,32.0]):
            c.observe_angular_velocity(i*.02,[0,0,0]);c.target(i*.02,a)
            self.assertEqual(c.observed[6],0)

    def test_edge_actions_stay_within_existing_pose_envelope(self):
        m=model();m['u_equilibrium']=[100,-100,100,-100]
        c=self.make(linear=m);c.hold_from_start=True;pose,_,_=c.target(0,32.1)
        self.assertLessEqual(abs(pose[0,3]),.025);self.assertLessEqual(abs(pose[1,3]),.025)


if __name__=='__main__':unittest.main()
