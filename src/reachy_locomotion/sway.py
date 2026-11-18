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

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("Gait values must be finite")
        if not .25 <= self.frequency_hz <= 5 or abs(self.roll_deg) > 40 or abs(self.yaw_deg) > 60:
            raise ValueError("Sway outside exploratory frequency/angle bounds")
        if abs(self.pitch_deg) + abs(self.pitch_bias_deg) > 40 or self.mirror not in (-1, 1):
            raise ValueError("Invalid pitch or mirror value")
        if max(abs(self.x_mm), abs(self.y_mm), abs(self.z_mm)) > 25:
            raise ValueError("Translation outside exploratory 25 mm bound")

    def pose(self, phase, envelope=1.0):
        r = self.mirror * self.roll_deg * math.sin(phase)
        y = self.mirror * self.yaw_deg * math.sin(phase + math.radians(self.yaw_phase_deg))
        p = self.pitch_bias_deg + self.pitch_deg * math.sin(2 * phase + math.radians(self.pitch_phase_deg))
        pose = np.eye(4)
        pose[:3, :3] = rotation(np.radians([r, p, y]) * envelope)
        pose[:3, 3] = [self.x_mm * envelope / 1000,
                      self.mirror * self.y_mm * envelope * math.sin(phase + math.radians(self.y_phase_deg)) / 1000,
                      .177 + self.z_mm * envelope / 1000]
        return pose


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
        self.initial_qpos = state.qpos.copy()
        self.initial_qvel = state.qvel.copy()
        self.initial_ctrl = state.ctrl.copy()
        if any(w.number for w in state.warning):
            raise ValueError("Numerical warning during neutral initialization")

    def checked_ik(self, pose):
        joints = self.ik.solve(pose)
        limits = self.model.actuator_ctrlrange[self.act]
        if np.any(joints < limits[:, 0]) or np.any(joints > limits[:, 1]):
            raise ValueError("IK exceeds inspected model joint limits")
        return joints

    def run(self, gait, cycles=8, warmup_cycles=2, record=False):
        if cycles < 1 or warmup_cycles < 1:
            raise ValueError("Require positive measurement and warm-up cycles")
        model = self.model
        state = mujoco.MjData(model)
        state.qpos[:] = self.initial_qpos
        state.qvel[:] = self.initial_qvel
        state.ctrl[:] = self.initial_ctrl
        mujoco.mj_forward(model, state)
        # Align endpoints to whole cycles AND 50 Hz control samples.
        frames_per_cycle = max(10, round(50 / gait.frequency_hz))
        actual_frequency = 50 / frames_per_cycle
        warmup_frames = warmup_cycles * frames_per_cycle
        total_frames = (warmup_cycles + cycles) * frames_per_cycle
        start = start_rot = None
        trajectory, boundaries = [], []
        reason = None
        max_tilt = max_closure = max_track = 0.0
        torque_saturated = measured_steps = foot_contact_steps = 0
        left_unloaded = right_unloaded = 0
        max_side_clearance = 0.0
        motor_work_j = 0.0
        initial_yaw = 0.0
        for frame in range(total_frames + 1):
            phase = 2 * math.pi * frame / frames_per_cycle
            ramp = min(1, frame / frames_per_cycle)
            envelope = ramp * ramp * (3 - 2 * ramp)
            desired = gait.pose(phase, envelope)
            try:
                target = self.checked_ik(desired)
            except ValueError as error:
                reason = str(error)
                break
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
                if (frame - warmup_frames) % frames_per_cycle == 0:
                    displacement = start_rot.T @ (state.xpos[self.base] - start)
                    boundaries.append({"cycle": (frame - warmup_frames) // frames_per_cycle,
                                       "foot_displacement_m": displacement.tolist()})
            if record:
                trajectory.append({"time_s": float(state.time), "measuring": measuring,
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
                    torque_saturated += int(np.any(np.abs(state.actuator_force[self.act]) >= .99 * self.config["motor_force_limit_nm"]))
                    motor_work_j += float(np.sum(np.abs(state.actuator_force[self.act] * state.actuator_velocity[self.act]))) * self.dt
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
                  "max_foot_tilt_deg": math.degrees(max_tilt), "max_closure_error_m": max_closure,
                  "max_head_orientation_tracking_error_deg": math.degrees(max_track),
                  "max_side_rim_clearance_m": max_side_clearance,
                  "foot_contact_fraction": foot_contact_steps/max(1, measured_steps),
                  "left_unloaded_fraction": left_unloaded/max(1, measured_steps),
                  "right_unloaded_fraction": right_unloaded/max(1, measured_steps),
                  "any_motor_torque_saturated_fraction": torque_saturated/max(1, measured_steps),
                  "absolute_motor_work_j": motor_work_j,
                  "cycle_boundaries": boundaries, "external_forces_applied": False,
                  "mujoco_version": mujoco.__version__, "control_rate_hz": 50,
                  "upstream_xml_sha256": self.source_sha256, "kinematics_sha256": self.ik.source_sha256}
        if record:
            result["trajectory"] = trajectory
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    parser.add_argument("--config", default="configs/contact-proxy.json")
    parser.add_argument("--gait", help="JSON file of Gait fields")
    parser.add_argument("--cycles", type=int, default=8)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Choose a new report path")
    gait = Gait(**json.loads(Path(args.gait).read_text())) if args.gait else Gait()
    sim = SwaySimulator(args.upstream, json.loads(Path(args.config).read_text()))
    result = sim.run(gait, cycles=args.cycles, record=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as file:
        json.dump(result, file, indent=2, allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ("trajectory", "cycle_boundaries", "config")}, indent=2))


if __name__ == "__main__":
    main()
