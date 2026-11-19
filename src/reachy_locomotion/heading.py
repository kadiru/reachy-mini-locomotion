"""Bounded head-stroke steering from a simulated external heading estimate.

The sensor is an explicit assumption, not a commissioned hardware interface.
No contact forces, privileged model parameters or base forces enter the policy.
"""
import math
from collections import deque
from dataclasses import asdict, dataclass, replace

import numpy as np


@dataclass
class HeadingFeedback:
    kp: float = .08
    ki: float = .04
    kd: float = 0.
    filter_tau_s: float = .25
    max_correction_deg: float = 3.
    correction_rate_deg_s: float = 8.
    yaw_per_roll: float = 0.
    roll_per_correction: float = 1.
    phase_skew_per_correction: float = 0.
    cycle_average: bool = False
    cycle_update: bool = False
    startup_ramp_cycles: float = 1.
    yaw_excursion_target_deg: float = 0.
    min_yaw_scale: float = .6
    initial_yaw_scale: float = 1.
    amplitude_adaptation_rate: float = .2
    gain_schedule: bool = False
    yaw_amplitude_response: float = 1.
    yaw_phase_slowdown_deg: float = 0.
    roll_slowdown_deg: float = 0.
    yaw_harmonic_per_correction: float = 0.
    yaw_harmonic_phase_deg: float = 0.
    phase_fit: bool = False
    heading_deadband_deg: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Feedback parameters must be finite')
        if not (0 <= self.kp <= 2 and 0 <= self.ki <= 2 and 0 <= self.kd <= 1):
            raise ValueError('Feedback gains outside exploratory bounds')
        if not (.02 <= self.filter_tau_s <= 2 and 0 < self.max_correction_deg <= 6
                and 0 < self.correction_rate_deg_s <= 40 and abs(self.yaw_per_roll) <= 5):
            raise ValueError('Feedback filter/correction outside exploratory bounds')
        if abs(self.roll_per_correction) > 1:
            raise ValueError('Roll correction scale outside exploratory bounds')
        if abs(self.phase_skew_per_correction) > 1:
            raise ValueError('Phase skew correction scale outside exploratory bounds')
        if (not isinstance(self.cycle_average, bool) or not isinstance(self.cycle_update, bool)
                or not 1 <= self.startup_ramp_cycles <= 6):
            raise ValueError('Invalid cycle averaging/startup ramp')
        if not (0 <= self.yaw_excursion_target_deg <= 10 and .4 <= self.min_yaw_scale <= self.initial_yaw_scale <= 1
                and 0 < self.amplitude_adaptation_rate <= 1 and isinstance(self.gain_schedule, bool)):
            raise ValueError('Invalid measured-excursion adaptation')
        if not (0 <= self.yaw_amplitude_response <= 1 and 0 <= self.yaw_phase_slowdown_deg <= 100
                and 0 <= self.roll_slowdown_deg <= 20):
            raise ValueError('Invalid stroke governor mapping')
        if abs(self.yaw_harmonic_per_correction) > 5 or abs(self.yaw_harmonic_phase_deg) > 180:
            raise ValueError('Invalid twice-per-stride yaw correction')
        if not isinstance(self.phase_fit,bool):
            raise ValueError('Phase estimator flag must be boolean')
        if not 0 <= self.heading_deadband_deg <= 20:
            raise ValueError('Heading deadband must lie in [0, 20] degrees')


@dataclass
class HeadingSensor:
    delay_frames: int = 3
    noise_std_deg: float = .25
    bias_deg: float = 0.
    seed: int = 19031

    def __post_init__(self):
        if (not isinstance(self.delay_frames, int) or not 0 <= self.delay_frames <= 50
                or not math.isfinite(self.noise_std_deg) or not 0 <= self.noise_std_deg <= 10
                or not math.isfinite(self.bias_deg) or abs(self.bias_deg) > 10):
            raise ValueError('Invalid simulated heading sensor')


class HeadingController:
    def __init__(self, feedback, sensor, initial_yaw_rad, cycle_frames=27):
        self.config, self.sensor = feedback, sensor
        self.previous_yaw = initial_yaw_rad
        self.unwrapped_deg = self.filtered_deg = self.integral = self.correction_deg = 0.
        self.queue = deque([0.] * (sensor.delay_frames + 1), maxlen=sensor.delay_frames + 1)
        self.rng = np.random.default_rng(sensor.seed)
        self.history = deque([0.] * cycle_frames, maxlen=cycle_frames)
        self.fit_history = deque(maxlen=2*cycle_frames)
        self.fit_rate = 0.
        self.control_heading = self.raw_command = 0.
        self.yaw_scale = self.minimum_yaw_scale = feedback.initial_yaw_scale
        self.gain_scale = 1.
        self.peak_correction_deg = 0.
        self.saturated_frames = self.frames = 0

    def update(self, yaw_rad, dt=.02):
        self.unwrapped_deg += math.degrees((yaw_rad-self.previous_yaw+math.pi) % (2*math.pi)-math.pi)
        self.previous_yaw = yaw_rad
        observed = self.unwrapped_deg + self.sensor.bias_deg + self.rng.normal(0, self.sensor.noise_std_deg)
        self.queue.append(observed)
        self.history.append(self.queue[0])
        self.fit_history.append(self.queue[0])
        c = self.config
        if self.frames >= 3*len(self.history) and self.frames % len(self.history) == 0:
            excursion = (max(self.history)-min(self.history))/2
            if c.yaw_excursion_target_deg:
                severity = max(excursion, abs(sum(self.history)/len(self.history)), .1)
                ratio = c.yaw_excursion_target_deg/severity
                change = float(np.clip(self.yaw_scale*(ratio**c.amplitude_adaptation_rate-1), -.1, .05))
                self.yaw_scale = float(np.clip(self.yaw_scale+change, c.min_yaw_scale, 1.))
                self.minimum_yaw_scale = min(self.minimum_yaw_scale, self.yaw_scale)
            if c.gain_schedule:
                self.gain_scale = float(np.clip(4./max(excursion,.1), .3, 1.))
        observation = sum(self.history)/len(self.history) if c.cycle_average else self.queue[0]
        if c.phase_fit and self.frames >= 2*len(self.history)+self.sensor.delay_frames:
            offsets=np.arange(-len(self.fit_history)+1,1)-self.sensor.delay_frames
            phases=(self.frames+offsets)*2*math.pi/len(self.history)
            # Fit drift plus periodic rocking in the delayed measurements.
            # Time zero is NOW: the intercept predicts current mean heading,
            # avoiding the extra half-cycle lag of a moving average. Raw yaw
            # remains the acceptance metric; this filter never hides failures.
            design=np.column_stack([np.ones(len(offsets)),offsets*dt,
                                    np.sin(phases),np.cos(phases),np.sin(2*phases),np.cos(2*phases)])
            fit=np.linalg.lstsq(design,np.asarray(self.fit_history),rcond=None)[0]
            observation=float(np.clip(fit[0],min(self.fit_history)-5,max(self.fit_history)+5))
            self.fit_rate=float(np.clip(fit[1],-60,60))
        self.filtered_deg += -math.expm1(-dt/c.filter_tau_s) * (observation-self.filtered_deg)
        if not c.cycle_update or self.frames % len(self.history) == 0:
            control_dt = dt*len(self.history) if c.cycle_update else dt
            error = math.copysign(max(0., abs(self.filtered_deg)-c.heading_deadband_deg), self.filtered_deg)
            rate = (error-self.control_heading)/control_dt
            if c.phase_fit and self.frames >= 2*len(self.history)+self.sensor.delay_frames:
                rate=self.fit_rate
            if c.heading_deadband_deg and not error:
                rate = 0.
            self.control_heading = error
            candidate_integral = self.integral + self.gain_scale*c.ki*error*control_dt
            raw = self.gain_scale*c.kp*error + candidate_integral + self.gain_scale*c.kd*rate
            # Positive head roll bias produced negative body yaw in the steering probe.
            # Conditional integration prevents winding up against the correction cap.
            if abs(raw) <= c.max_correction_deg or raw*error < 0:
                self.integral = candidate_integral
            self.raw_command = self.gain_scale*c.kp*error + self.integral + self.gain_scale*c.kd*rate
        raw = self.raw_command
        bounded = float(np.clip(raw, -c.max_correction_deg, c.max_correction_deg))
        self.correction_deg += float(np.clip(bounded-self.correction_deg,
                                            -c.correction_rate_deg_s*dt, c.correction_rate_deg_s*dt))
        self.peak_correction_deg = max(self.peak_correction_deg, abs(self.correction_deg))
        self.saturated_frames += int(abs(raw) >= c.max_correction_deg)
        self.frames += 1
        return self.correction_deg

    def steer(self, gait):
        roll_bias = float(np.clip(gait.roll_bias_deg + gait.mirror*self.config.roll_per_correction*self.correction_deg, -40, 40))
        yaw_bias = float(np.clip(gait.yaw_bias_deg + gait.mirror*self.config.yaw_per_roll*self.correction_deg, -60, 60))
        harmonic = float(np.clip(gait.yaw_harmonic_deg+gait.mirror*self.config.yaw_harmonic_per_correction*self.correction_deg,
                                 -min(15.,60-abs(yaw_bias)), min(15.,60-abs(yaw_bias))))
        # Bias and amplitude share the original pose envelope. IK and the motor
        # target-rate/torque model still apply to every resulting command.
        return replace(gait, roll_bias_deg=roll_bias, yaw_bias_deg=yaw_bias,
                       yaw_harmonic_deg=harmonic, yaw_harmonic_phase_deg=gait.yaw_harmonic_phase_deg+self.config.yaw_harmonic_phase_deg,
                       phase_skew=float(np.clip(gait.phase_skew+gait.mirror*self.config.phase_skew_per_correction*self.correction_deg, -.5, .5)),
                       roll_deg=math.copysign(min(max(0.,abs(gait.roll_deg)-self.config.roll_slowdown_deg*(1-self.yaw_scale)), 40-abs(roll_bias)), gait.roll_deg),
                       yaw_phase_deg=gait.yaw_phase_deg+self.config.yaw_phase_slowdown_deg*(1-self.yaw_scale),
                       yaw_deg=math.copysign(min(abs(gait.yaw_deg)*(1-self.config.yaw_amplitude_response*(1-self.yaw_scale)), 60-abs(yaw_bias)-abs(harmonic)), gait.yaw_deg))

    def report(self):
        return dict(config=asdict(self.config), sensor=asdict(self.sensor),
                    sensing_assumption='simulated external foot heading, relative to startup; not hardware calibrated',
                    peak_correction_deg=self.peak_correction_deg,
                    minimum_yaw_scale=self.minimum_yaw_scale,
                    saturated_fraction=self.saturated_frames/max(1, self.frames))
