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
    @unittest.skipUnless(filter_available(), 'Requires the optional IMU filter')
    def test_packet_clocks_and_hold_between_motor_ticks(self):
        from multirate_imu import Packets
        from types import SimpleNamespace
        model = SimpleNamespace(opt=SimpleNamespace(timestep=.00025, gravity=np.array([0.,0.,-9.80665])))
        signals = {'balance_accelerometer':np.array([0.,0.,9.80665]), 'balance_gyro':np.array([0.,0.,1.])}
        data = SimpleNamespace(site=lambda _:SimpleNamespace(xmat=np.eye(3)),
                               sensor=lambda name:SimpleNamespace(data=signals[name]))
        for rate, phase, expected in [(25,0,[0,1,1,2]),(25,.5,[1,1,2,2]),(1000,0,[20,40,60,80])]:
            bank = Packets(rate,phase)
            bank.sample(model,data,0.)
            counts = []
            headings = []
            for tick in range(1,5):
                for step in range(80):
                    bank.advance()
                bank.sample(model,data,tick*.02)
                counts.append(bank.packets)
                headings.append(bank.heading_deg)
            self.assertEqual(counts, expected)
            self.assertGreater(headings[-1],3.)
            if rate == 25 and phase == 0:
                self.assertEqual(headings[1],headings[2])

    @unittest.skipUnless(filter_available(), 'Requires the optional IMU filter')
    def test_head_motion_removed_from_stationary_foot(self):
        from unittest.mock import patch
        from multirate_imu import Packets
        estimator = Packets(200,0,mount='head+FK')
        pose = np.eye(4)
        estimator.q = np.array([1.,0.,0.,0.])
        with patch('multirate_imu.relative_head_pose', return_value=pose):
            estimator.sample(None,None,0.)
        theta = .02
        pose = pose.copy()
        pose[:2,:2] = [[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]
        estimator.q[:] = [math.cos(theta/2),0.,0.,math.sin(theta/2)]
        estimator.last_rate = np.array([0.,0.,1.])
        with patch('multirate_imu.relative_head_pose', return_value=pose):
            attitude, gyro = estimator.sample(None,None,.02)
        np.testing.assert_allclose(attitude,0.,atol=1e-10)
        np.testing.assert_allclose(gyro,0.,atol=1e-10)
        self.assertAlmostEqual(estimator.heading_deg,0.)

    def test_relative_heading_crosses_wrap_without_changing_direction(self):
        from multirate_imu import RelativeYaw
        estimator = RelativeYaw()
        for degrees in (178.,-179.,-176.):
            rad = math.radians(degrees/2)
            estimator.angles(np.array([math.cos(rad),0.,0.,math.sin(rad)]))
        self.assertAlmostEqual(estimator.heading_deg,6.)

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
