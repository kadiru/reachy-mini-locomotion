import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from imu_reference_catch import ReferenceCatch
from reachy_locomotion.geometry import euler


class ReferenceCatchTest(unittest.TestCase):
    def test_feedback_and_replay_share_the_plan_but_respond_differently_to_tilt(self):
        template = (np.array([[2., -3.]]), np.array([[31., -1.]]), np.array([[33., 1.]]))
        params = dict(kp=3., kd=.3, deadband=.2, shift=.3)
        plant = SimpleNamespace(checked_ik=lambda pose: None)
        with patch('imu_reference_catch.reference', return_value=template):
            active = ReferenceCatch(plant, 1201, params)
            replay = ReferenceCatch(plant, 1201, dict(params, kp=0., kd=0.))
        for regulator in (active, replay):
            regulator.observe_angular_velocity(0., np.zeros(3))
        np.testing.assert_array_equal(active.target(0., 32.)[0], replay.target(0., 32.)[0])
        corrected = active.target(.02, 35.)[0]
        nominal = replay.target(.02, 35.)[0]
        self.assertGreater(math.degrees(euler(corrected)[0]), 6.)
        self.assertAlmostEqual(math.degrees(euler(nominal)[0]), 2.)
        self.assertEqual((active.corrections, replay.corrections), (1, 0))
