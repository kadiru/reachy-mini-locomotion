"""Head-actuated sway experiments in the uncalibrated moving-foot model."""
import argparse
import copy
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from .contact_probe import BASE, initialize, relative_head_pose, scene_xml
from .geometry import angle, euler, rotation


@dataclass
class Gait:
    frequency_hz: float = 2.0
    roll_deg: float = 20.0
    yaw_deg: float = 20.0
    yaw_phase_deg: float = 90.0
    pitch_deg: float = 0.0
    pitch_phase_deg: float = 0.0
    pitch_bias_deg: float = 0.0
    x_mm: float = 0.0
    y_mm: float = 0.0
    y_phase_deg: float = 0.0
    z_mm: float = 0.0
    mirror: float = 1.0
    roll_sharpness: float = 0.0
    yaw_sharpness: float = 0.0
    roll_bias_deg: float = 0.0
    yaw_bias_deg: float = 0.0
    phase_skew: float = 0.0
    yaw_harmonic_deg: float = 0.0
    yaw_harmonic_phase_deg: float = 0.0
    yaw_stroke_fraction: float = 0.0
    roll_stroke_fraction: float = 0.0
    stroke_accel_fraction: float = 0.20
    stroke_brake_fraction: float = 0.05

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("Gait values must be finite")
        # Steering subtracts bias/harmonic amplitudes from these shared bounds.
        # Floating-point addition can recover 60.00000000000001 from that clip.
        # Tolerate arithmetic roundoff only; motor/IK limits are still enforced.
        if not .25 <= self.frequency_hz <= 5 or abs(self.roll_deg) + abs(self.roll_bias_deg) > 40+1e-12 or abs(self.yaw_deg) + abs(self.yaw_bias_deg) + abs(self.yaw_harmonic_deg) > 60+1e-12:
            raise ValueError("Sway outside exploratory frequency/angle bounds")
        if not 0 <= self.roll_sharpness <= 5 or not 0 <= self.yaw_sharpness <= 5:
            raise ValueError("Swing sharpness must lie in [0, 5]")
        if abs(self.phase_skew) > .5:
            raise ValueError("Phase skew outside monotonic stroke bound")
        if any(v != 0 and not .15 <= v <= 1 for v in (self.yaw_stroke_fraction, self.roll_stroke_fraction)):
            raise ValueError("Stroke fraction must be zero (legacy) or in [.15, 1]")
        if not (.025 <= self.stroke_accel_fraction <= .45 and .025 <= self.stroke_brake_fraction <= .45):
            raise ValueError("Stroke acceleration/braking fractions must lie in [.025, .45]")
        if abs(self.pitch_deg) + abs(self.pitch_bias_deg) > 40 or self.mirror not in (-1, 1):
            raise ValueError("Invalid pitch or mirror value")
        if max(abs(self.x_mm), abs(self.y_mm), abs(self.z_mm)) > 25:
            raise ValueError("Translation outside exploratory 25 mm bound")

    def pose(self, phase, envelope=1.0):
        # Warp timing within each cycle while preserving its period and endpoints.
        # d(warped phase)/d(phase) >= .5: no reverse or discontinuous strokes.
        phase = phase + self.phase_skew * (1-math.cos(phase))
        # Smooth, bounded swing-and-hold. Large sharpness shortens the stroke
        # and lengthens its end dwell. It never changes motor torque/rate caps.
        def swing(p, sharpness, stroke):
            if stroke:
                return brake_stroke(p, stroke, self.stroke_accel_fraction, self.stroke_brake_fraction)
            return math.sin(p) if sharpness == 0 else math.tanh(sharpness * math.sin(p)) / math.tanh(sharpness)
        r = self.mirror * (self.roll_bias_deg + self.roll_deg * swing(phase, self.roll_sharpness, self.roll_stroke_fraction))
        y = self.mirror * (self.yaw_bias_deg + self.yaw_deg * swing(phase + math.radians(self.yaw_phase_deg), self.yaw_sharpness, self.yaw_stroke_fraction))
        y += self.mirror*self.yaw_harmonic_deg*math.sin(2*phase+math.radians(self.yaw_harmonic_phase_deg))
        p = self.pitch_bias_deg + self.pitch_deg * math.sin(2 * phase + math.radians(self.pitch_phase_deg))
        pose = np.eye(4)
        pose[:3, :3] = rotation(np.radians([r, p, y]) * envelope)
        pose[:3, 3] = [self.x_mm * envelope / 1000,
                      self.mirror * self.y_mm * envelope * math.sin(phase + math.radians(self.y_phase_deg)) / 1000,
                      .177 + self.z_mm * envelope / 1000]
        return pose


def brake_stroke(phase, move_fraction, accel_fraction, brake_fraction):
    """Continuous position/velocity, trapezoidal speed, then an endpoint hold.

    Fractions describe a target waveform, not additional motor capability.
    All targets still pass through IK, 50 Hz sampling and the same bounded servo.
    Zero phase crosses the stroke centre approximately, as for the legacy sine.
    """
    half = ((phase + math.pi / 2) % (2 * math.pi)) / math.pi
    u = float(np.clip(((half % 1) - (1 - move_fraction) / 2) / move_fraction, 0, 1))
    area = 1 - (accel_fraction + brake_fraction) / 2
    if u < accel_fraction:
        progress = u * u / (2 * accel_fraction * area)
    elif u <= 1 - brake_fraction:
        progress = (u - accel_fraction / 2) / area
    else:
        progress = 1 - (1 - u) ** 2 / (2 * brake_fraction * area)
    return (2 * progress - 1) * (1 if half < 1 else -1)


class HeadIK:
    """Use the official Rust engine directly, without importing a robot client.

    Input is T_foot_head in the raw MJCF frame, including absolute head height.
    This is the SDK analytical mapping before its 177 mm origin subtraction.
    """
    def __init__(self, upstream):
        from reachy_mini_rust_kinematics import ReachyMiniRustKinematics
        path = Path(upstream) / "src/reachy_mini/assets/kinematics_data.json"
        raw = path.read_bytes()
        self.source_sha256 = hashlib.sha256(raw).hexdigest()
        config = json.loads(raw)
        self.engine = ReachyMiniRustKinematics(config["motor_arm_length"], config["rod_length"])
        self.names = ["yaw_body"] + [branch["name"] for branch in config["motors"]]
        for branch in config["motors"]:
            self.engine.add_branch(branch["branch_position"], np.linalg.inv(branch["T_motor_world"]).tolist(),
                                   1 if branch["solution"] else -1)

    def solve(self, pose):
        try:
            joints = np.r_[0.0, self.engine.inverse_kinematics(pose.tolist(), 0.0)]
        except (ValueError, RuntimeError) as error:
            raise ValueError("Unreachable head pose: " + str(error)) from error
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("IK returned invalid joints")
        return joints


class SwaySimulator:
    def __init__(self, upstream, config):
        self.config = copy.deepcopy(config)
        xml, self.source_sha256 = scene_xml(upstream, config)
        fixed_xml, _ = scene_xml(upstream, config, free=False)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.ik = HeadIK(upstream)
        self.act = np.array([self.model.actuator(name).id for name in self.ik.names])
        self.command_limits = self.model.actuator_ctrlrange[self.act].copy()
        self.servos = None
        self.joint_qpos = self.model.jnt_qposadr[self.model.actuator_trnid[self.act, 0]]
        self.base = self.model.body(BASE).id
        self.foot = self.model.geom("foot_contact_proxy").id
        self.floor = self.model.geom("floor").id
        self.closing_a = [self.model.site("closing_%d_1" % i).id for i in range(1, 6)]
        self.closing_b = [self.model.site("closing_%d_2" % i).id for i in range(1, 6)]
        self.dt = float(self.model.opt.timestep)
        self.substeps = round(.02 / self.dt)
        if not math.isclose(self.substeps * self.dt, .02, abs_tol=1e-12):
            raise ValueError("Timestep must divide 20 ms control period")
        state = initialize(self.model, config, fixed_xml)
        neutral = np.eye(4)
        neutral[2, 3] = .177
        neutral_joints = self.checked_ik(neutral)
        previous = state.ctrl[self.act].copy()
        for step in range(round(2 / self.dt)):
            blend = min(1, step * self.dt)
            blend = blend * blend * (3 - 2 * blend)
            state.ctrl[self.act] = previous * (1 - blend) + neutral_joints * blend
            mujoco.mj_step(self.model, state)
        mujoco.mj_forward(self.model, state)
        if "motor_model" in config:
            from .motors import DynamixelApproximation
            self.servos = DynamixelApproximation(self.model, config["motor_model"])
            self.initial_targets = state.ctrl.copy()
            self.servos.reset(self.initial_targets)
            for _ in range(round(2 / self.dt)):
                self.servos.apply(state)
                mujoco.mj_step(self.model, state)
            mujoco.mj_forward(self.model, state)
        self.initial_qpos = state.qpos.copy()
        self.initial_qvel = state.qvel.copy()
        self.initial_ctrl = state.ctrl.copy()
        if any(w.number for w in state.warning):
            raise ValueError("Numerical warning during neutral initialization")

    def checked_ik(self, pose):
        joints = self.ik.solve(pose)
        limits = self.command_limits
        if np.any(joints < limits[:, 0]) or np.any(joints > limits[:, 1]):
            raise ValueError("IK exceeds inspected model joint limits")
        return joints

    def run(self, gait, cycles=8, warmup_cycles=2, record=False, feedback=None, sensor=None):
        if cycles < 1 or warmup_cycles < 1:
            raise ValueError("Require positive measurement and warm-up cycles")
        model = self.model
        state = mujoco.MjData(model)
        state.qpos[:] = self.initial_qpos
        state.qvel[:] = self.initial_qvel
        state.ctrl[:] = self.initial_ctrl
        if self.servos:
            self.servos.reset(self.initial_targets)
        mujoco.mj_forward(model, state)
        startup_pos = state.xpos[self.base].copy()
        startup_yaw = math.atan2(state.xmat[self.base].reshape(3, 3)[1, 0], state.xmat[self.base].reshape(3, 3)[0, 0])
        startup_rotation = rotation([0, 0, startup_yaw])
        startup_previous_yaw = startup_yaw
        startup_heading = max_startup_heading = 0.
        controller = None
        if feedback is not None:
            from .heading import HeadingController, HeadingFeedback, HeadingSensor
            controller = HeadingController(HeadingFeedback(**feedback), HeadingSensor(**(sensor or {})), startup_yaw,
                                           max(10, round(50 / gait.frequency_hz)))
        # Align endpoints to whole cycles AND 50 Hz control samples.
        frames_per_cycle = max(10, round(50 / gait.frequency_hz))
        actual_frequency = 50 / frames_per_cycle
        warmup_frames = warmup_cycles * frames_per_cycle
        total_frames = (warmup_cycles + cycles) * frames_per_cycle
        start = start_rot = None
        trajectory, boundaries = [], []
        reason = None
        max_tilt = max_closure = max_track = 0.0
        max_heading = max_lateral = 0.0
        previous_heading = unwrapped_heading = 0.0
        torque_saturated = measured_steps = foot_contact_steps = 0
        left_unloaded = right_unloaded = 0
        max_side_clearance = 0.0
        motor_work_j = 0.0
        braking_work_j = 0.0
        airborne_steps = flight_run_steps = longest_flight_steps = flight_count = 0
        max_foot_bottom_clearance = 0.0
        minimum_joint_margin = float("inf")
        initial_yaw = 0.0
        velocity_samples = []
        peak_torque = np.zeros(model.nu)
        torque_sq = np.zeros(model.nu)
        peak_velocity = np.zeros(model.nu)
        for frame in range(total_frames + 1):
            phase = 2 * math.pi * frame / frames_per_cycle
            ramp_cycles = controller.config.startup_ramp_cycles if controller else 1.
            ramp = min(1, frame / (frames_per_cycle * ramp_cycles))
            envelope = ramp * ramp * (3 - 2 * ramp)
            current_yaw = math.atan2(state.xmat[self.base].reshape(3, 3)[1, 0], state.xmat[self.base].reshape(3, 3)[0, 0])
            startup_heading += (current_yaw-startup_previous_yaw+math.pi) % (2*math.pi)-math.pi
            startup_previous_yaw = current_yaw
            max_startup_heading = max(max_startup_heading, abs(startup_heading))
            if controller:
                controller.update(current_yaw)
            commanded_gait = controller.steer(gait) if controller else gait
            desired = commanded_gait.pose(phase, envelope)
            try:
                target = self.checked_ik(desired)
            except ValueError as error:
                reason = str(error)
                break
            if self.servos:
                targets = self.initial_targets.copy()
                targets[self.act] = target
                self.servos.command(targets)
            else:
                state.ctrl[self.act] = target
            if frame == warmup_frames:
                start = state.xpos[self.base].copy()
                start_rot = state.xmat[self.base].reshape(3, 3).copy()
                initial_yaw = math.atan2(start_rot[1, 0], start_rot[0, 0])
                # Flatten the evaluation frame; vertical tilt must not count as travel.
                start_rot = rotation([0, 0, initial_yaw])
            measuring = frame >= warmup_frames
            tilt = math.acos(float(np.clip(state.xmat[self.base].reshape(3, 3)[2, 2], -1, 1)))
            closure = float(np.max(np.linalg.norm(state.site_xpos[self.closing_a] - state.site_xpos[self.closing_b], axis=1)))
            max_tilt, max_closure = max(max_tilt, tilt), max(max_closure, closure)
            actual = relative_head_pose(model, state)
            if measuring:
                max_track = max(max_track, angle(desired, actual))
                velocity_samples.append(state.qvel[:6].copy())
                heading = math.atan2(state.xmat[self.base].reshape(3, 3)[1, 0], state.xmat[self.base].reshape(3, 3)[0, 0])
                if frame == warmup_frames:
                    previous_heading = heading
                unwrapped_heading += (heading - previous_heading + math.pi) % (2 * math.pi) - math.pi
                previous_heading = heading
                max_heading = max(max_heading, abs(unwrapped_heading))
                displacement = start_rot.T @ (state.xpos[self.base] - start)
                max_lateral = max(max_lateral, abs(float(displacement[1])))
                if (frame - warmup_frames) % frames_per_cycle == 0:
                    displacement = start_rot.T @ (state.xpos[self.base] - start)
                    boundaries.append({"cycle": (frame - warmup_frames) // frames_per_cycle,
                                       "foot_displacement_m": displacement.tolist()})
            if record:
                mujoco.mj_subtreeVel(model, state)
                head_body = int(model.site_bodyid[model.site("head").id])
                side_forces = [0.0, 0.0]
                for ci, contact in enumerate(state.contact):
                    if {int(contact.geom1), int(contact.geom2)} == {self.foot, self.floor} and contact.efc_address >= 0:
                        wrench = np.zeros(6)
                        mujoco.mj_contactForce(model, state, ci, wrench)
                        point = state.xmat[self.base].reshape(3, 3).T @ (contact.pos - state.xpos[self.base])
                        side_forces[int(point[1] >= 0)] += abs(float(wrench[0]))
                trajectory.append({"time_s": float(state.time), "measuring": measuring,
                                   "heading_from_startup_rad": startup_heading,
                                   "heading_estimate_deg": controller.filtered_deg if controller else None,
                                   "steering_correction_deg": controller.correction_deg if controller else 0.,
                                   "yaw_amplitude_scale": controller.yaw_scale if controller else 1.,
                                   "phase_rad": phase % (2 * math.pi),
                                   "foot_side_normal_forces_n_right_left": side_forces,
                                   "qvel": state.qvel.tolist(),
                                   "motor_torque_nm": state.actuator_force.tolist(),
                                   "motor_velocity_rad_s": state.actuator_velocity.tolist(),
                                   "head_subtree_angular_momentum_world": state.subtree_angmom[head_body].tolist(),
                                   "head_subtree_com_world_m": state.subtree_com[head_body].tolist(),
                                   "qpos": state.qpos.tolist(), "ctrl": state.ctrl.tolist(),
                                   "foot_xyz_m": state.xpos[self.base].tolist(),
                                   "foot_rpy_rad": euler(state.xmat[self.base].reshape(3, 3)).tolist(),
                                   "head_rpy_rad": euler(actual).tolist(),
                                   "target_head_rpy_rad": euler(desired).tolist()})
            if tilt > math.radians(35):
                reason = "foot_tilt_exceeded_35_deg"
                break
            if closure > .001:
                reason = "loop_closure_error_exceeded_1_mm"
                break
            if frame == total_frames:
                break
            for _ in range(self.substeps):
                # Only servo targets act on the robot. No base motion writes or pushes.
                if self.servos:
                    self.servos.apply(state)
                mujoco.mj_step(model, state)
                mujoco.mj_forward(model, state)
                if not np.isfinite(state.qpos).all() or not np.isfinite(state.qvel).all() or any(w.number for w in state.warning):
                    reason = "numerical_warning"
                    break
                tilt = math.acos(float(np.clip(state.xmat[self.base].reshape(3, 3)[2, 2], -1, 1)))
                closure = float(np.max(np.linalg.norm(state.site_xpos[self.closing_a] - state.site_xpos[self.closing_b], axis=1)))
                max_tilt, max_closure = max(max_tilt, tilt), max(max_closure, closure)
                if tilt > math.radians(35):
                    reason = "foot_tilt_exceeded_35_deg"
                if closure > .001:
                    reason = "loop_closure_error_exceeded_1_mm"
                forces = [0.0, 0.0]
                supported = False
                for contact_index, contact in enumerate(state.contact):
                    pair = {int(contact.geom1), int(contact.geom2)}
                    if self.floor in pair and self.foot not in pair and contact.efc_address >= 0:
                        reason = "nonfoot_floor_contact"
                    if pair == {self.foot, self.floor} and contact.efc_address >= 0:
                        supported = True
                        wrench = np.zeros(6)
                        mujoco.mj_contactForce(model, state, contact_index, wrench)
                        foot_point = state.xmat[self.base].reshape(3, 3).T @ (contact.pos - state.xpos[self.base])
                        forces[int(foot_point[1] >= 0)] += abs(float(wrench[0]))
                if measuring:
                    measured_steps += 1
                    foot_contact_steps += int(supported)
                    left_unloaded += int(forces[1] < .1 and forces[0] > .1)
                    right_unloaded += int(forces[0] < .1 and forces[1] > .1)
                    caps = self.servos.torque_caps[self.act] if self.servos else self.config["motor_force_limit_nm"]
                    torque_saturated += int(np.any(np.abs(state.actuator_force[self.act]) >= .99 * caps))
                    peak_torque = np.maximum(peak_torque, np.abs(state.actuator_force))
                    torque_sq += state.actuator_force ** 2
                    peak_velocity = np.maximum(peak_velocity, np.abs(state.actuator_velocity))
                    motor_work_j += float(np.sum(np.abs(state.actuator_force[self.act] * state.actuator_velocity[self.act]))) * self.dt
                    power = state.actuator_force[self.act] * state.actuator_velocity[self.act]
                    braking_work_j += float(np.sum(np.maximum(-power, 0))) * self.dt
                    # Cylinder support function: a raised rim is not a flight.
                    row_z = state.geom_xmat[self.foot].reshape(3, 3)[2]
                    bottom = float(state.geom_xpos[self.foot, 2] -
                                   self.config["foot_half_height_m"] * abs(row_z[2]) -
                                   self.config["foot_radius_m"] * np.linalg.norm(row_z[:2]))
                    max_foot_bottom_clearance = max(max_foot_bottom_clearance, bottom)
                    airborne = bottom > .0001 and sum(forces) < .05
                    if airborne:
                        flight_count += int(flight_run_steps == 0)
                        flight_run_steps += 1
                        airborne_steps += 1
                        longest_flight_steps = max(longest_flight_steps, flight_run_steps)
                    else:
                        flight_run_steps = 0
                    q = state.qpos[self.joint_qpos]
                    minimum_joint_margin = min(minimum_joint_margin, float(np.min(q-self.command_limits[:, 0])),
                                               float(np.min(self.command_limits[:, 1]-q)))
                    for sign in (-1, 1):
                        rim = state.xpos[self.base] + state.xmat[self.base].reshape(3, 3) @ np.array([0, sign*self.config["foot_radius_m"], 0])
                        max_side_clearance = max(max_side_clearance, float(rim[2]))
                if reason:
                    break
            if reason:
                break
        displacement = start_rot.T @ (state.xpos[self.base] - start) if start is not None else np.zeros(3)
        measured_seconds = measured_steps * self.dt
        yaw = math.atan2(state.xmat[self.base].reshape(3, 3)[1, 0], state.xmat[self.base].reshape(3, 3)[0, 0])
        result = {"evidence": "uncalibrated_simulation", "gait": asdict(gait), "actual_frequency_hz": actual_frequency,
                  "config": self.config, "valid": reason is None, "failure_reason": reason,
                  "requested_cycles": cycles, "warmup_cycles": warmup_cycles,
                  "measured_seconds": measured_seconds, "forward_m": float(displacement[0]),
                  "lateral_m": float(displacement[1]), "heading_change_rad": (yaw-initial_yaw+math.pi)%(2*math.pi)-math.pi,
                  "forward_m_per_s": float(displacement[0])/measured_seconds if measured_seconds else 0.0,
                  "max_heading_excursion_rad": max_heading,
                  "max_heading_from_startup_rad": max_startup_heading,
                  "final_heading_from_startup_rad": startup_heading,
                  "startup_frame_displacement_m": (startup_rotation.T @ (state.xpos[self.base]-startup_pos)).tolist(),
                  "unwrapped_heading_change_rad": unwrapped_heading,
                  "max_lateral_excursion_m": max_lateral,
                  "max_foot_tilt_deg": math.degrees(max_tilt), "max_closure_error_m": max_closure,
                  "max_head_orientation_tracking_error_deg": math.degrees(max_track),
                  "max_side_rim_clearance_m": max_side_clearance,
                  "foot_contact_fraction": foot_contact_steps/max(1, measured_steps),
                  "left_unloaded_fraction": left_unloaded/max(1, measured_steps),
                  "right_unloaded_fraction": right_unloaded/max(1, measured_steps),
                  "any_motor_torque_saturated_fraction": torque_saturated/max(1, measured_steps),
                  "absolute_motor_work_j": motor_work_j,
                  "negative_motor_work_j": braking_work_j,
                  "airborne_fraction": airborne_steps/max(1, measured_steps),
                  "longest_airborne_interval_s": longest_flight_steps*self.dt,
                  "airborne_intervals": flight_count,
                  "airborne_definition": "foot-cylinder bottom >0.1 mm and summed foot normal force <0.05 N",
                  "max_foot_bottom_clearance_m": max_foot_bottom_clearance,
                  "min_motor_joint_limit_margin_rad": minimum_joint_margin if measured_steps else None,
                  "measurement_displacement_in_startup_frame_m": (startup_rotation.T @ (state.xpos[self.base]-start)).tolist() if start is not None else [0., 0., 0.],
                  "cycle_boundaries": boundaries, "external_forces_applied": False,
                  "mujoco_version": mujoco.__version__, "control_rate_hz": 50,
                  "upstream_xml_sha256": self.source_sha256, "kinematics_sha256": self.ik.source_sha256}
        result["actuator_names"] = [model.actuator(i).name for i in range(model.nu)]
        result["peak_motor_torque_nm"] = peak_torque.tolist()
        result["rms_motor_torque_nm"] = np.sqrt(torque_sq / max(1, measured_steps)).tolist()
        result["peak_motor_velocity_rad_s"] = peak_velocity.tolist()
        result["base_subtree_mass_kg"] = float(model.body_subtreemass[self.base])
        velocities = np.asarray(velocity_samples)
        if len(velocities) > 2:
            accelerations = np.diff(velocities[:, :3], axis=0) / .02
            result["rms_base_acceleration_m_s2"] = float(np.sqrt(np.mean(accelerations ** 2)))
            result["rms_base_angular_velocity_rad_s"] = float(np.sqrt(np.mean(velocities[:, 3:] ** 2)))
        if self.servos:
            result["motor_diagnostics"] = self.servos.report()
        if controller:
            result["heading_feedback"] = controller.report()
        if record:
            result["trajectory"] = trajectory
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    parser.add_argument("--config", default="configs/motors-datasheet.json")
    parser.add_argument("--gait", help="JSON file of Gait fields")
    parser.add_argument("--cycles", type=int, default=8)
    parser.add_argument("--feedback", help="JSON file of HeadingFeedback fields")
    parser.add_argument("--sensor", help="JSON file of simulated HeadingSensor fields")
    parser.add_argument("--warmup-cycles", type=int, default=2)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Choose a new report path")
    gait = Gait(**json.loads(Path(args.gait).read_text())) if args.gait else Gait()
    sim = SwaySimulator(args.upstream, json.loads(Path(args.config).read_text()))
    result = sim.run(gait, cycles=args.cycles, warmup_cycles=args.warmup_cycles, record=True,
                     feedback=json.loads(Path(args.feedback).read_text()) if args.feedback else None,
                     sensor=json.loads(Path(args.sensor).read_text()) if args.sensor else None)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as file:
        json.dump(result, file, indent=2, allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ("trajectory", "cycle_boundaries", "config")}, indent=2))


if __name__ == "__main__":
    main()
