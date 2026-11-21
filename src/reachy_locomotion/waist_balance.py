"""A bounded body-yaw sweep after natural capture on one side of the foot."""
import math

import numpy as np

from .edge_linear import LinearEdgeController
from .geometry import rotation
from .primitives import Primitive


class WaistBalance(LinearEdgeController):
    """Keep the learned balance coordinates aligned with the supporting foot."""

    def __init__(self, policy, side, checked_ik, amplitude_deg, period_s, *,
                 direction=1, compensate=True, freeze_head=False, com_offset_mm=0., gain_change=0., return_fraction=.5):
        super().__init__(Primitive(**policy['recipe']), policy['feedback'], side,
                         self.check_virtual_pose, linear=policy['linear'])
        self.solve_pose = checked_ik
        self.control_gain = policy.get('linear_gain_scale', 1.)
        if not 0 <= amplitude_deg <= 159 or not 2 <= period_s <= 180 or direction not in (-1, 1):
            raise ValueError('Invalid bounded waist sweep')
        self.amplitude = math.radians(amplitude_deg) * direction
        self.period = period_s
        self.compensate = compensate
        self.freeze_head = freeze_head
        if not -15 <= com_offset_mm <= 15:
            raise ValueError('Invalid center-of-mass feedforward offset')
        self.com_offset_mm = com_offset_mm
        if not -.6 <= gain_change <= .6 or not .15 <= return_fraction <= .85:
            raise ValueError('Invalid angular gain or sweep timing')
        self.base_gain = self.control_gain
        self.gain_change = gain_change
        self.return_fraction = return_fraction
        self.waist_target_rad = 0.
        self.encoder = 0.
        self.stable_frames = 0
        self.start = None
        self.hold_pose = None

    def observe_waist(self, position_rad):
        self.encoder = position_rad

    def physical_pose(self, virtual, yaw):
        if not yaw:
            return virtual
        rz = rotation((0., 0., yaw))
        pose = virtual.copy()
        if self.compensate:
            # Keep the tilt axis and translations in the foot frame as the
            # head faces around with the torso. The IK input is foot-to-head.
            pose[:3, :3] = virtual[:3, :3] @ rz
            if self.com_offset_mm:
                # Feedforward for a fore/aft mass offset revolving with the torso.
                pose[0, 3] += .001 * self.com_offset_mm * (1 - math.cos(yaw))
                pose[1, 3] -= .001 * self.com_offset_mm * math.sin(yaw)
        else:
            pose[:3, :] = rz @ virtual[:3, :]
        return pose

    def check_virtual_pose(self, pose):
        return self.solve_pose(self.physical_pose(pose, self.waist_target_rad), self.waist_target_rad)

    def observe_head_state(self, t, pose):
        if self.start is not None and self.compensate:
            pose = pose.copy()
            pose[:3, :3] = pose[:3, :3] @ rotation((0., 0., -self.encoder))
        super().observe_head_state(t, pose)

    def target(self, t, roll_deg):
        if self.start is None:
            stable = (self.catch_trigger is not None and abs(self.sensed[0] - self.side*self.f.target_roll_deg) < 2
                      and abs(self.rate[0]) < 5 and abs(self.pitch) < 5)
            self.stable_frames = self.stable_frames + 1 if stable else 0
            if self.stable_frames >= 25:
                self.start = t
        if self.start is not None:
            elapsed = t - self.start
            phase = 2 * math.pi * elapsed / self.period
            # Smoothly enter a continuous sinusoid without a velocity jump.
            ramp = min(1., elapsed / min(1., self.period/8))
            wave = math.sin(phase)
            if self.return_fraction != .5:
                outbound = 1 - self.return_fraction
                progress = (elapsed / self.period + outbound / 2) % 1
                if progress < outbound:
                    wave = -math.cos(math.pi * progress / outbound)
                else:
                    wave = math.cos(math.pi * (progress - outbound) / self.return_fraction)
            self.waist_target_rad = self.amplitude * wave * ramp*ramp*(3-2*ramp)
            self.control_gain = self.base_gain + self.gain_change * (1 - math.cos(self.encoder)) / 2
        pose, stage, sensed = super().target(t, roll_deg)
        if self.start is None:
            return pose, stage, sensed
        if self.freeze_head:
            if self.hold_pose is None:
                self.hold_pose = pose.copy()
            # A hold relative to the turning torso fixes the Stewart joint targets.
            rz = rotation((0., 0., self.waist_target_rad))
            pose = self.hold_pose.copy()
            pose[:3, :] = rz @ self.hold_pose[:3, :]
        else:
            pose = self.physical_pose(pose, self.waist_target_rad)
        return pose, 'same-edge waist sweep', sensed
