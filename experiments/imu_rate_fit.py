"""Learn an interval-rate estimate from causal IMU and relative head observations."""
from collections import deque

import numpy as np

from imu_transfer import FilteredPackets
from moving_table import POLICY, plant, read
from reachy_locomotion.contact_probe import relative_head_pose
from reachy_locomotion.geometry import euler
from reachy_locomotion.primitives import Primitive
from train_release_catch import make_controller


class RatePackets(FilteredPackets):
    def __init__(self, age, weights=None):
        super().__init__(age)
        self.weights = np.asarray(weights) if weights is not None else None
        self.history = deque(maxlen=4)
        self.examples = []

    def sample(self, model, data, t):
        tilt, gyro = super().sample(model, data, t)
        head = relative_head_pose(model, data)
        frame = np.r_[self.trace[-1][3:9], np.degrees(euler(head)[0]), 1000 * head[1, 3]]
        if not self.history:
            self.history.extend([frame] * 3)
        self.history.append(frame)
        # Include the previous three packets and head displacements over each interval.
        observation = np.r_[1., np.concatenate([f[:6] for f in self.history]),
                            np.diff(np.array(self.history)[:, 6:], axis=0).ravel()]
        if self.weights is None:
            self.examples.append(observation)
            return None, None  # Teacher alone controls the offline demonstration.
        rate = gyro.copy()
        delta = np.clip(observation @ self.weights, -80., 80.)
        rate[0] += np.radians(delta)
        return tilt, rate


def demonstrate(condition):
    angle, rate, seed, age = condition
    sim = plant(.000125, True)
    packets = RatePackets(age)
    teacher = make_controller(sim, 1, seed, read(POLICY)['sides']['1'])
    teacher_rates = []
    observe = teacher.observe_angular_velocity

    def record(t, gyro):
        teacher_rates.append(np.degrees(gyro[0]))
        observe(t, gyro)

    teacher.observe_angular_velocity = record
    result = sim.run_primitive(Primitive(), seconds=6., controller=teacher, waist_mode='hold_neutral',
                               diagnostic_initial_roll_deg=angle, diagnostic_initial_angular_rate_deg_s=rate,
                               tilt_limit_deg=None, observe_landing=True, imu_observer=packets)
    relevant = np.array([abs(r['roll_deg'] - 32.1) < 5. for r in result['trace']])
    measured = np.asarray(packets.trace)[:, 6]
    return np.asarray(packets.examples)[relevant], (teacher_rates - measured)[relevant]


def coefficients(x, y, penalty):
    # Coefficients act on unscaled features on the robot. Scale only for the ridge fit.
    scale = np.maximum(x.std(axis=0), .01)
    scale[0] = 1.
    scaled = x / scale
    # Some Accelerate BLAS builds set floating-point flags even for finite products.
    with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
        gram = scaled.T @ scaled
        rhs = scaled.T @ y
    if not np.isfinite(gram).all() or not np.isfinite(rhs).all():
        raise ValueError('Non-finite training features or interval-rate targets')
    weights = np.linalg.solve(gram + penalty * np.eye(x.shape[1]), rhs) / scale
    if not np.isfinite(weights).all():
        raise ValueError('Non-finite observer fit')
    return weights.tolist()
