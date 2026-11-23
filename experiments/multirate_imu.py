"""A timestamped simulated IMU feeding fixed-rate trained controllers."""
import math
from contextlib import contextmanager, ExitStack
from unittest.mock import patch

import mujoco
import numpy as np

from imu_transfer import counts, FilteredPackets, GRAVITY, supported_quaternion
from reachy_locomotion.geometry import euler
from reachy_locomotion.contact_probe import relative_head_pose
from reachy_locomotion.primitives import PrimitiveSimulator
from reachy_locomotion.sway import SwaySimulator


class RelativeYaw:
    heading_deg = 0.
    previous_yaw = None

    def angles(self, quat):
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, quat)
        roll, pitch, yaw = np.degrees(euler(rotation.reshape(3, 3)))
        if self.previous_yaw is not None:
            self.heading_deg += (yaw - self.previous_yaw + 180.) % 360. - 180.
        self.previous_yaw = yaw
        return np.array([roll, pitch])


class StockPackets(FilteredPackets, RelativeYaw):
    def sample(self, model, data, t):
        result = super().sample(model, data, t)
        self.angles(self.q)
        return result


class Packets(RelativeYaw):
    """Integrate each fresh packet once and aggregate angular rate for each motor tick."""
    def __init__(self, hz, phase, mount="base", band="wide"):
        self.acc_bw, self.gyro_bw = {"wide": (145,116), "narrow": (40,12)}[band]
        self.mount = mount
        self.previous_relative = None
        self.previous_control = 0.
        from ahrs.filters import Madgwick
        self.hz, self.phase = hz, phase
        self.filter = Madgwick()
        self.q = None
        self.ticks = self.packets = self.in_window = 0
        self.last_time = 0.
        self.sum_rate = np.zeros(3)
        self.last_rate = np.zeros(3)

    def sample(self, model, data, t):
        if self.q is None:
            self.dt = model.opt.timestep
            if self.hz * self.dt > 1.000001:
                raise ValueError('IMU clock exceeds the physics rate')
            prefix, site = ('balance_', 'balance_imu') if self.mount == 'base' else ('moving_head_', 'head')
            self.raw_acc = data.sensor(prefix + 'accelerometer').data
            self.raw_gyro = data.sensor(prefix + 'gyro').data
            self.acc = data.site(site).xmat.reshape(3,3).T @ -model.opt.gravity
            self.rate = np.zeros(3)
            self.q = supported_quaternion(counts(self.acc / GRAVITY, 10920.))
            self.acc_gain = -math.expm1(-2 * math.pi * self.acc_bw * self.dt)
            self.rate_gain = -math.expm1(-2 * math.pi * self.gyro_bw * self.dt)
            self.next_time = (1. - self.phase) / self.hz
        average = self.sum_rate / self.in_window if self.in_window else self.last_rate
        self.sum_rate[:] = 0.
        self.in_window = 0
        if self.mount != 'head+FK':
            return self.angles(self.q), average
        # Diagnostic upper bound. These relative poses represent perfect, synchronized
        # forward kinematics. They contain no absolute attitude or angular velocity.
        relative = relative_head_pose(model, data)[:3,:3]
        head_world = np.empty(9)
        mujoco.mju_quat2Mat(head_world,self.q)
        base_q = np.empty(4)
        mujoco.mju_mat2Quat(base_q, (head_world.reshape(3,3) @ relative.T).ravel())
        foot_rate = relative @ average
        if self.previous_relative is not None and t > self.previous_control:
            dq = np.empty(4)
            mujoco.mju_mat2Quat(dq,(relative @ self.previous_relative.T).ravel())
            relative_rate = np.empty(3)
            mujoco.mju_quat2Vel(relative_rate,dq,t-self.previous_control)
            foot_rate -= relative_rate
        self.previous_relative = relative
        self.previous_control = t
        return self.angles(base_q), foot_rate

    def advance(self):
        self.ticks += 1
        now = self.ticks * self.dt
        limit = math.radians(500.)
        self.acc += self.acc_gain * (np.clip(self.raw_acc, -3*GRAVITY, 3*GRAVITY) - self.acc)
        self.rate += self.rate_gain * (np.clip(self.raw_gyro, -limit, limit) - self.rate)
        if now + 1e-10 < self.next_time:
            return
        self.last_rate = np.radians(counts(np.degrees(self.rate), 65.536))
        self.q = self.filter.updateIMU(self.q, gyr=self.last_rate, acc=counts(self.acc/GRAVITY, 10920.),
                                       dt=now - self.last_time)
        self.sum_rate += self.last_rate
        self.in_window += 1
        self.packets += 1
        self.last_time = now
        self.next_time = (self.packets + 1. - self.phase) / self.hz


@contextmanager
def feed_archived_runner(rate, phase, mount="base", include_head=False, band="wide"):
    # Scope injection to the serial runner inside an isolated process. Archived jobs
    # keep their own plants, contact geometry, actuator parameters, scoring and seeds.
    streams = []
    init = SwaySimulator.__init__
    def instrument(sim, upstream, config):
        init(sim, upstream, dict(config, base_imu=True, head_imu=include_head or mount != "base"))

    def inject(method):
        def wrapped(sim, *args, **kwargs):
            if rate != 'archive':
                observer = StockPackets(phase * 10) if rate == 'stock' else Packets(rate, phase, mount, band)
                streams.append(observer)
                kwargs['imu_observer'] = observer
            return method(sim, *args, **kwargs)
        return wrapped

    with ExitStack() as stack:
        stack.enter_context(patch.object(SwaySimulator, '__init__', instrument))
        for cls, name in ((PrimitiveSimulator,'run_primitive'), (SwaySimulator,'run')):
            stack.enter_context(patch.object(cls, name, inject(getattr(cls, name))))
        yield streams
