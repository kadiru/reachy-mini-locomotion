"""Periodic head motion for sustained free-base rotation in simulation."""
from collections import deque
from dataclasses import astuple, dataclass
import math

import numpy as np

from .geometry import rotation
from .sway import brake_stroke


@dataclass(frozen=True)
class Orbit:
    period_frames: int
    roll_deg: float
    pitch_deg: float
    pitch_phase_deg: float
    xy_mm: float
    xy_phase_deg: float
    yaw_deg: float
    yaw_phase_deg: float
    z_mm: float
    pitch_bias_deg: float = 0.
    yaw_harmonic: int = 1
    xy_sense: int = 1
    vertical_mm: float = 0.
    vertical_phase_deg: float = 0.
    yaw_stroke_fraction: float = 0.
    rate_gain: float = 0.
    reference_deg_s: float = 24.
    roll_bias_deg: float = 0.
    y_bias_mm: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in astuple(self)):
            raise ValueError('Orbit parameters must be finite')
        if abs(self.rate_gain)>.35 or not 5<=self.reference_deg_s<=65:
            raise ValueError('Invalid spin-rate feedback')
        if not (10 <= self.period_frames <= 75 and self.period_frames == int(self.period_frames)):
            raise ValueError('Period outside the 50 Hz command grid')
        if (abs(self.roll_deg)+abs(self.roll_bias_deg)>40 or abs(self.pitch_deg)+abs(self.pitch_bias_deg)>40 or
                abs(self.xy_mm)+abs(self.y_bias_mm)>25 or abs(self.z_mm)+abs(self.vertical_mm)>25 or abs(self.yaw_deg)>60 or
                self.yaw_harmonic not in (1,2) or self.xy_sense not in (-1,1) or
                (self.yaw_stroke_fraction and not .15<=self.yaw_stroke_fraction<=1.)):
            raise ValueError('Head motion exceeds the simulated command workspace')


class SpinController:
    """A repeated head orbit, with one ramp from the neutral upright start."""
    catch_trigger = yaw_trigger = None

    def __init__(self, orbit, checked_ik, limits, mirror=1, *, direction=1, delay=1, noise=0., seed=0, hold_at=None):
        if mirror not in (-1,1) or direction not in (-1,1):
            raise ValueError('Mirror and spin direction must each be -1 or 1')
        self.orbit = orbit
        self.checked_ik = checked_ik
        self.limits = limits
        self.mirror = mirror
        self.cache = {}
        self.direction = direction
        self.heading = deque([0.]*(delay+1),maxlen=delay+1)
        self.previous_heading = 0.
        self.rate = 0.
        self.phase = 0.
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.hold_at = hold_at
        self.last_pose = None

    def observe_heading(self, t, degrees):
        if not self.orbit.rate_gain:
            return
        self.heading.append(degrees+self.rng.normal(0.,self.noise))
        sensed=self.heading[0]
        self.rate += (1-math.exp(-.02/.16))*(self.direction*(sensed-self.previous_heading)/.02-self.rate)
        self.previous_heading=sensed

    def target(self, t, roll):
        if self.hold_at is not None and t>=self.hold_at and self.last_pose is not None:
            return self.last_pose, 'fixed head target', 0.
        frame = round(t*50)
        tick = frame % self.orbit.period_frames
        ramp = min(frame/50, 1.)
        if not self.orbit.rate_gain and ramp == 1. and tick in self.cache:
            self.last_pose=self.cache[tick]
            return self.last_pose, 'orbit', 0.
        o = self.orbit
        phi = 2*math.pi*tick/o.period_frames
        if o.rate_gain:
            correction = o.rate_gain*(o.reference_deg_s-self.rate)/o.reference_deg_s
            if frame:
                self.phase += 2*math.pi/o.period_frames*(1+float(np.clip(correction,-.18,.18)))
            phi = self.phase
        target = self.pose_for_phase(phi, ramp*ramp*(3-2*ramp))
        self.last_pose=target
        if not o.rate_gain and ramp==1.:
            self.cache[tick] = target
        return target, 'orbit', 0.

    def pose_for_phase(self, phi, envelope):
        o = self.orbit
        pp, xp, yp = map(math.radians, (o.pitch_phase_deg,o.xy_phase_deg,o.yaw_phase_deg))
        r = self.mirror*(o.roll_bias_deg+o.roll_deg*math.cos(phi))
        p = o.pitch_bias_deg + o.pitch_deg*math.cos(phi+pp)
        yaw_phase = o.yaw_harmonic*phi+yp
        yaw_shape = (brake_stroke(yaw_phase+math.pi/2, o.yaw_stroke_fraction, .20, .05)
                     if o.yaw_stroke_fraction else math.cos(yaw_phase))
        y = self.mirror*o.yaw_deg*yaw_shape
        angles = np.radians([r,p,y])
        height = o.z_mm + o.vertical_mm*math.cos(o.yaw_harmonic*phi+math.radians(o.vertical_phase_deg))
        offset = .001*np.array([o.xy_sense*o.xy_mm*math.sin(phi+xp),self.mirror*(o.y_bias_mm+o.xy_mm*math.cos(phi+xp)),height])

        def pose(scale):
            result = np.eye(4)
            result[:3,:3] = rotation(angles*scale)
            result[:3,3] = offset*scale + [0,0,.177]
            return result

        def allowed(target):
            try:
                joints = self.checked_ik(target)
            except ValueError:
                return False
            return bool(np.all(joints>=self.limits[:,0]+.08) and np.all(joints<=self.limits[:,1]-.08))

        target = pose(envelope)
        if not allowed(target):
            low,high = 0.,envelope
            for _ in range(8):
                mid = (low+high)/2
                if allowed(pose(mid)):
                    low = mid
                else:
                    high = mid
            target = pose(low)
        return target


def spin_metrics(result, duration, direction):
    """Measure signed unwrapped rotation, frame continuity and center excursion."""
    rows = result['trace']
    times = np.array([r['time_s'] for r in rows])
    base = np.array([r['foot_yaw_deg'] for r in rows])
    torso = np.array([r['torso_yaw_deg'] for r in rows])
    xy = np.array([r['foot_xyz_m'][:2] for r in rows])
    distance = 1000 * np.linalg.norm(xy - xy[0], axis=1)
    dt = np.diff(times)
    signed_step = direction * np.diff(base)

    # Check overlapping half-second windows every 20 ms after the startup ramp.
    after_ramp = np.flatnonzero(times >= 1.)
    if len(after_ramp) > 25:
        end = after_ramp[25:]
        window_rates = direction * (base[end] - base[end - 25]) / .5
    else:
        window_rates = np.array([0.])
    forward_windows = float(np.mean(window_rates > 2.))
    post = times[1:] >= 1.
    moving = signed_step > 2 * dt
    forward_frames = float(dt[post & moving].sum() / max(.02, dt[post].sum()))

    last_half = times >= max(1., duration / 2)
    late_rate = 0.
    if last_half.sum() > 1:
        late_rate = direction * (base[-1] - base[last_half][0]) / max(.02, times[-1] - times[last_half][0])

    side = np.array([r['edge_support'] for r in rows])
    fore = np.array([r['foreaft_edge_support'] for r in rows])
    roll = np.array([r['roll_deg'] for r in rows])
    pitch = np.array([r['pitch_deg'] for r in rows])
    edge = side | fore
    supported = edge[1:] & edge[:-1]
    same_rim = ((side[1:] & side[:-1] & (roll[1:] * roll[:-1] > 0))
                | (fore[1:] & fore[:-1] & (pitch[1:] * pitch[:-1] > 0)))

    longest_edge = longest_pause = edge_streak = pause_streak = 0.
    for i, elapsed in enumerate(dt):
        edge_streak = edge_streak + elapsed if supported[i] else 0.
        pause_streak = pause_streak + elapsed if post[i] and not moving[i] else 0.
        longest_edge = max(longest_edge, edge_streak)
        longest_pause = max(longest_pause, pause_streak)

    measured_seconds = max(.02, times[-1])
    speed = direction * base[-1] / measured_seconds
    passed = (result['completed'] and result['min_joint_margin_rad'] >= 0
              and min(direction * base[-1], direction * torso[-1]) >= 720
              and forward_windows >= .98 and forward_frames >= .80
              and distance.max() <= 56 and late_rate > 0)
    return dict(
        seconds=duration, elapsed_s=float(times[-1]), direction=direction,
        base_deg=float(base[-1]), torso_deg=float(torso[-1]), rpm=float(speed / 6),
        rim_base_deg=float(np.diff(base)[same_rim].sum()),
        rim_torso_deg=float(np.diff(torso)[same_rim].sum()), last_half_deg_s=float(late_rate),
        min_half_second_deg_s=float(window_rates.min()), forward_window_fraction=forward_windows,
        forward_frame_fraction=forward_frames, reverse_deg=float(-np.minimum(signed_step, 0.).sum()),
        max_excursion_mm=float(distance.max()), edge_fraction=float(dt[supported].sum()) / measured_seconds,
        longest_edge_s=float(longest_edge), longest_pause_s=float(longest_pause),
        airborne_fraction=float(np.mean([r['normal_n'] < .1 for r in rows])),
        peak_tilt_deg=result['peak_tilt_deg'], waist_command_rad=result['commanded_waist_position_range_rad'],
        joint_margin_rad=result['min_joint_margin_rad'], termination=result['termination'],
        fell=result['fell'], passed=bool(passed),
    )
