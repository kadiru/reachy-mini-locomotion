import math
import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from imu_transfer import FilteredPackets, IMUPackets, counts


def filter_available():
    try:
        from ahrs.filters import Madgwick
    except ImportError:
        return False
    except TypeError:
        # The upstream package evaluates PEP 604 annotations at import time.
        if sys.version_info >= (3,10):
            raise
        return False
    return Madgwick is not None


class BalanceIMUTest(unittest.TestCase):
    @unittest.skipUnless(filter_available(),
                         'Requires the optional IMU filter on Python 3.10 or newer')
    def test_calibration_orientation_and_dynamic_specific_force(self):
        tilt = math.radians(-22.)
        model = mujoco.MjModel.from_xml_string(f"""
            <mujoco><worldbody><body><joint type="slide" axis="0 1 0"/>
            <geom type="sphere" size=".02" mass="1"/>
            <site name="balance_imu" quat="{math.cos(tilt/2)} {math.sin(tilt/2)} 0 0"/>
            </body></worldbody><sensor>
            <accelerometer site="balance_imu" name="balance_accelerometer"/>
            <gyro site="balance_imu" name="balance_gyro"/></sensor></mujoco>""")
        d = mujoco.MjData(model)
        mujoco.mj_forward(model,d)
        tracker = IMUPackets('raw')
        attitude, gyro = tracker.sample(model,d,0.)
        np.testing.assert_allclose(attitude, [-22.,0.], atol=.01)
        np.testing.assert_allclose(gyro, 0.)
        initial = np.array(tracker.trace[-1][3:6])
        d.qfrc_applied[0] = 3.
        mujoco.mj_forward(model,d)
        tracker.sample(model,d,.02)
        expected = np.array([0.,math.cos(tilt),-math.sin(tilt)])*3./9.80665
        np.testing.assert_allclose(np.array(tracker.trace[-1][3:6])-initial,expected,atol=.0002)
        handoff = IMUPackets('handoff')
        self.assertEqual(handoff.sample(model,d,0.),(None,None))
        estimate, _ = handoff.sample(model,d,3.)
        np.testing.assert_allclose(estimate,[-22.,0.],atol=.01)

    @unittest.skipUnless(filter_available(),
                         'Requires the optional IMU filter on Python 3.10 or newer')
    def test_filtered_register_is_held_between_100_hz_updates(self):
        model = mujoco.MjModel.from_xml_string("""
            <mujoco><option timestep=".000125"/><worldbody><body><joint type="hinge"/>
            <geom type="sphere" size=".02" mass="1"/><site name="balance_imu"/>
            </body></worldbody><sensor>
            <accelerometer site="balance_imu" name="balance_accelerometer"/>
            <gyro site="balance_imu" name="balance_gyro"/></sensor></mujoco>""")
        data = mujoco.MjData(model)
        data.qvel[0] = math.radians(60.)
        mujoco.mj_forward(model,data)
        tracker = FilteredPackets(5)
        tracker.channels(model,data)
        for _ in range(39):
            tracker.advance()
        self.assertEqual(tracker.register[1][2],0.)
        tracker.advance()
        first=60.*(1.-math.exp(-2.*math.pi*12.*.005))
        self.assertAlmostEqual(tracker.register[1][2],first)
        for _ in range(79):
            tracker.advance()
        self.assertAlmostEqual(tracker.register[1][2],first)
        tracker.advance()
        self.assertGreater(tracker.register[1][2],first)

    def test_register_saturation(self):
        np.testing.assert_allclose(counts(np.array([-800.,0.,800.]),65.536),[-500.,0.,32767/65.536])
