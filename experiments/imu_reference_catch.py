"""Track a successful level-ground head trajectory using live tilt and gyro residuals."""
import math
from collections import deque
from functools import lru_cache
from pathlib import Path

import numpy as np

from imu_transfer import FilteredPackets
from moving_table import POLICY, plant, read, write
from reachy_locomotion.geometry import euler, rotation
from reachy_locomotion.primitives import Primitive
from train_release_catch import make_controller

TEMPLATE = Path('configs/imu-right-reference.json')


class ShadowPackets(FilteredPackets):
    def sample(self, model, data, t):
        super().sample(model, data, t)
        return None, None


def teach(age):
    sim = plant(.000125, True)
    sensor = ShadowPackets(age)
    teacher = make_controller(sim, 1, 4915, read(POLICY)['sides']['1'])
    result = sim.run_primitive(Primitive(), seconds=8., controller=teacher, waist_mode='hold_neutral',
                               diagnostic_initial_roll_deg=32.2, tilt_limit_deg=None, observe_landing=True,
                               imu_observer=sensor)
    if result['first_body_floor_contact'] or not result['completed']:
        raise ValueError('Incomplete teacher demonstration')
    actions = np.array([r['plane_action'] for r in result['trace']])
    head = np.array([r['plane_state'][1:3] for r in result['trace']])
    return actions, np.asarray(sensor.trace)[:, [1, 6]], head


def build(pool):
    runs = list(pool.map(teach, (0, 5, 9)))
    for actions, _, _ in runs:
        np.testing.assert_array_equal(actions, runs[0][0])
    observations = np.stack([obs for _, obs, _ in runs])
    write(TEMPLATE, dict(duration=8., actions=runs[0][0].tolist(),
                         low=observations.min(axis=0).tolist(), high=observations.max(axis=0).tolist(),
                         head=runs[0][2].tolist()))
    reference.cache_clear()


@lru_cache(maxsize=1)
def reference():
    data = read(TEMPLATE)
    return tuple(np.asarray(data[k]) for k in ('actions', 'low', 'high'))


class ReferenceCatch:
    catch_trigger = yaw_trigger = None

    def __init__(self, sim, seed, params):
        self.sim, self.params = sim, params
        self.rng = np.random.default_rng(seed)
        self.actions, self.low, self.high = reference()
        self.corrections = 0

    def observe_angular_velocity(self, t, velocity):
        self.rate = math.degrees(velocity[0])

    def adjustment(self, i, error):
        correction = np.clip(self.params['kp'] * error[0] + self.params['kd'] * error[1], -16., 16.)
        return correction * np.array([1., self.params['shift']])

    def target(self, t, roll):
        i = min(round(t * 50), len(self.actions) - 1)
        sensed = [roll + self.rng.normal(0., .05), self.rate + self.rng.normal(0., .1)]
        pad = np.array([self.params['deadband'], .5])
        error = sensed - np.clip(sensed, self.low[i] - pad, self.high[i] + pad)
        correction = self.adjustment(i, error)
        self.corrections += int(np.abs(correction).max() > 1e-6)
        head_roll, shift = self.actions[i] + correction
        pose = np.eye(4)
        for scale in (1., .85, .65, .4, 0.):
            pose[:3, :3] = rotation((math.radians(scale * head_roll), 0., 0.))
            pose[:3, 3] = [0., scale * shift / 1000., .177]
            try:
                self.sim.checked_ik(pose)
                break
            except ValueError:
                if scale == 0.:
                    raise
        return pose, 'IMU reference tracking', sensed[0]


class LinkedReferenceCatch(ReferenceCatch):
    def __init__(self, sim, seed, params):
        super().__init__(sim, seed, params)
        model = read(POLICY)['sides']['1']['specialists'][3]['model']
        self.A, self.B, self.K = (np.array(model[key]) for key in ('A', 'B', 'K'))
        self.plan_head = np.array(read(TEMPLATE)['head'])
        self.previous = None
        self.delta_u = deque([np.zeros(2) for _ in range(3)], maxlen=3)

    def observe_head_state(self, t, pose):
        self.head = np.array([math.degrees(euler(pose)[0]), 1000 * pose[1, 3]])

    def adjustment(self, i, error):
        delta_head = self.head - self.plan_head[i]
        velocity = (delta_head - self.previous[1:3]) / .02 if self.previous is not None else np.zeros(2)
        current = np.r_[error[0], delta_head, self.params['rate_weight'] * error[1], velocity]
        if self.previous is None:
            prediction = np.r_[current, np.zeros(4)]
        else:
            prediction = self.A @ np.r_[self.previous, self.delta_u[0], self.delta_u[1]] + self.B @ self.delta_u[-1]
        target = -self.params['gain'] * (self.K @ prediction)
        step = self.params['slew'] * .02 * np.array([1., .65])
        action = np.clip(self.delta_u[-1] + np.clip(target - self.delta_u[-1], -step, step), -20., 20.)
        self.previous = current
        self.delta_u.append(action)
        return action
