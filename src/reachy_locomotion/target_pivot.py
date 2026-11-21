"""A heading-conditioned policy composed of measured head-driven rim pivots."""
from collections import deque

import numpy as np

from .balance import BalanceController, mirrored


class TargetPivot:
    """Select a recovered stroke using the remaining signed heading error."""

    def __init__(self, bank, target_deg, checked_ik, *, seed=0, delay=1,
                 noise=.12, max_strokes=9, deadband=4.1, single=False, delayed_edge_weight=3., repeat_margin=.065):
        if not np.isfinite(target_deg) or abs(target_deg) > 180:
            raise ValueError('Requested heading must be within 180 degrees of startup')
        self.bank = bank
        self.target_deg = target_deg
        self.checked_ik = checked_ik
        self.rng = np.random.default_rng(seed)
        self.history = deque([0.] * (delay + 1), maxlen=delay + 1)
        self.rest_roll = deque([0.] * (delay + 1), maxlen=delay + 1)
        self.noise = noise
        self.delay = delay
        self.seed = seed
        self.max_strokes = max_strokes
        self.deadband = deadband
        self.single = single
        self.delayed_edge_weight = delayed_edge_weight
        self.repeat_margin = repeat_margin
        self.heading = self.head = 0.
        self.child = None
        self.catch_trigger = self.yaw_trigger = None
        self.finished = False
        self.strokes = []
        self.measured_turns = {}
        self.responses = {o['id']: o['responses'] for o in bank if 'responses' in o}
        if not single and abs(target_deg) > deadband and len(self.responses) != len(bank):
            raise ValueError('Each deployed stroke needs a learned response table')
        if not bank and (single or abs(target_deg) > deadband):
            raise ValueError('A nonzero turn requires trained strokes')
        self.neutral = np.eye(4)
        self.neutral[2, 3] = .177

    def observe_heading(self, t, deg):
        self.history.append(deg + self.rng.normal(0, self.noise))
        self.heading = self.history[0]

    def observe_head(self, t, deg):
        self.head = deg

    def choose(self, error):
        # Infer the response from measured landings. The policy receives neither
        # the model condition nor any contact-force or ground-friction input.
        responses = next(iter(self.responses.values()))
        losses = np.array([0. if r['delay'] == self.delay else 4. for r in responses])
        for stroke in self.strokes[-3:]:
            measured = stroke['measured_turn_deg']
            for i, r in enumerate(self.responses[stroke['option']]):
                losses[i] += min(5., abs(measured - r['turn']) / 2.5)
                losses[i] += min(3., abs(stroke['peak_sensed_roll_deg'] - r['roll']) / 3.)
        weights = np.exp(-(losses - losses.min()))
        weights /= weights.sum()
        direction = np.sign(error)

        # With two delayed frames the slick-floor training trajectories often
        # turn during touchdown. Give the measured single-rim turn more weight.
        edge_weight = self.delayed_edge_weight if self.delay >= 2 else .65

        def cost(option):
            expected = self.responses[option['id']]
            turns = np.array([r['turn'] for r in expected])
            if option['id'] in self.measured_turns:
                turns = .25 * turns + .75 * self.measured_turns[option['id']]
            edge_shortfall = [max(0., direction * (.6*r['turn'] - r['edge'])) for r in expected]
            drift = [r['drift'] for r in expected]
            return weights @ (abs(error-turns) + edge_weight*np.array(edge_shortfall) + .25*np.array(drift))

        return min(self.bank, key=cost)

    def target(self, t, roll):
        self.rest_roll.append(roll)
        # Each burst includes a neutral return followed by a quiet observation window.
        if self.child is not None and t - self.started >= 4.4 - 1e-8:
            measured = self.heading - self.start_heading
            name = self.option['id']
            self.strokes[-1].update(end_s=t, measured_turn_deg=measured)
            self.measured_turns[name] = .3*self.measured_turns.get(name, measured) + .7*measured
            self.child = None
        if self.child is None and not self.finished:
            error = self.target_deg - self.heading
            if len(self.strokes) >= self.max_strokes or (not self.single and abs(error) <= self.deadband):
                self.finished = True
            else:
                self.option = self.bank[0] if self.single else self.choose(error)
                if self.option is None:
                    self.finished = True
                    return self.neutral.copy(), 'no qualified stroke', self.rest_roll[0]
                feedback = dict(self.option['feedback'], sensor_seed=self.seed + len(self.strokes),
                                sensor_delay_frames=self.delay, sensor_noise_deg=self.noise)
                if any(abs(stroke['measured_turn_deg']) > 12.5 for stroke in self.strokes):
                    feedback['workspace_joint_margin_rad'] = max(self.repeat_margin, feedback['workspace_joint_margin_rad'])
                self.child = BalanceController(mirrored(self.option['recipe'], self.option['side']),
                                               feedback, checked_ik=self.checked_ik)
                self.started = t
                self.start_heading = self.heading
                self.strokes.append(dict(option=self.option['id'], start_s=t, residual_deg=error, peak_sensed_roll_deg=0.))
        if self.child is None:
            return self.neutral.copy(), 'heading hold', self.rest_roll[0]
        local = t - self.started
        if local >= 3.7:
            return self.neutral.copy(), 'settling', self.rest_roll[0]
        self.child.observe_head(local, self.head)
        pose, stage, sensed = self.child.target(local, roll)
        self.strokes[-1]['peak_sensed_roll_deg'] = max(self.strokes[-1]['peak_sensed_roll_deg'], abs(sensed))
        if self.catch_trigger is None and self.child.catch_trigger is not None:
            self.catch_trigger = self.started + self.child.catch_trigger
        if self.yaw_trigger is None and self.child.yaw_trigger is not None:
            self.yaw_trigger = self.started + self.child.yaw_trigger
        return pose, stage, sensed


def edge_turns(rows, field):
    """Signed rotation while consecutive samples remain on the same loaded rim."""
    signed = {'left': 0., 'right': 0.}
    for a, b in zip(rows, rows[1:]):
        if a['edge_support'] and b['edge_support'] and a['roll_deg'] * b['roll_deg'] > 0:
            rim = 'right' if b['roll_deg'] > 0 else 'left'
            signed[rim] += b[field] - a[field]
    return signed


def pivot_metrics(result, target_deg):
    rows = result['trace']
    xy = np.array([row['foot_xyz_m'][:2] for row in rows])
    excursion = 1000 * np.linalg.norm(xy - xy[0], axis=1)
    signed = edge_turns(rows, 'foot_yaw_deg')
    tail = [r for r in rows if r['time_s'] >= rows[-1]['time_s'] - .6]
    error = max(abs(r[k] - target_deg) for r in tail for k in ('torso_yaw_deg', 'foot_yaw_deg'))
    return dict(target_deg=target_deg, torso_deg=result['final_torso_yaw_deg'],
                base_deg=result['final_foot_yaw_deg'], max_tail_error_deg=error,
                signed_edge_deg=sum(signed.values()), edge_deg_by_rim=signed,
                edge_s=result['edge_support_seconds'], lift_mm=result['peak_side_height_difference_mm'],
                slip_mm=result['edge_contact_slip_distance_mm'], max_excursion_mm=float(max(excursion)),
                drift_mm=float(excursion[-1]), waist_range_deg=result['measured_waist_angle_range_deg'],
                waist_command_rad=result['commanded_waist_position_range_rad'],
                elapsed_s=rows[-1]['time_s'], settled=result['settled'], fell=result['fell'],
                termination=result['termination'],
                passed=bool(result['completed'] and result['settled'] and error <= 5 and max(excursion) <= 56
                            and np.sign(target_deg) * sum(signed.values()) >= .4 * abs(target_deg)))
