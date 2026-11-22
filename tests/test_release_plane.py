import unittest
import numpy as np

from reachy_locomotion.release_plane import ReleaseSelector


class Probe:
    def __init__(self, index):
        self.index = index

    def observe_head_state(self,t,pose):
        self.head = pose

    def observe_angular_velocity(self,t,velocity):
        self.rate = velocity

    def observe_attitude(self,t,pitch):
        self.pitch = pitch

    def target(self,t,roll):
        return self.index

    def diagnostics(self):
        return {}


class ReleaseSelectorTests(unittest.TestCase):
    def test_selects_once_from_measured_angle_and_signed_outward_rate(self):
        config = dict(lead_s=.1, centers_deg=[31.,33.], specialists=[0,1])
        for side in (1,-1):
            for rate, expected in ((4.,1),(-4.,0)):
                selector = ReleaseSelector(side,123,config,Probe)
                selector.observe_attitude(0.,0.)
                selector.observe_head_state(0.,np.eye(4))
                selector.observe_angular_velocity(0.,np.radians([side*rate,0.,0.]))
                self.assertEqual(selector.target(0.,side*32.),expected)
                self.assertEqual(selector.target(.02,side*40.),expected)
                np.testing.assert_allclose(selector.controller.head,np.eye(4))
