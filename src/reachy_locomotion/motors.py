"""Datasheet-bounded DC/PWM approximation, NOT identified Dynamixel firmware.

See docs/MOTOR_LIMITS.md for provenance, inferred dynamics and missing telemetry.
Only this module's synthetic torques enter MuJoCo; no robot connection exists.
"""
import math
from collections import deque

import numpy as np

# (voltage, stall Nm, no-load rpm). Manufacturer values, not continuous ratings.
SPECS = {
    "XL330-M288": [(3.7, .42, 76), (5., .52, 103), (6., .60, 123)],
    "XL330-M077": [(3.7, .180, 278), (5., .215, 383), (6., .228, 456)],
    # Reachy's PG plastic gear variant is not independently characterized.
    "XC330-M288-proxy": [(3.7, .69, 59), (5., .93, 81), (6., 1.10, 97)],
}


def specification(name, voltage):
    if not math.isfinite(voltage) or not 3.7 <= voltage <= 6:
        raise ValueError("No extrapolation outside the manufacturer's 3.7–6 V data")
    rows = np.array(SPECS[name])
    return (float(np.interp(voltage, rows[:, 0], rows[:, 1])),
            float(np.interp(voltage, rows[:, 0], rows[:, 2])) * 2 * math.pi / 60)


def validate_motor_config(c):
    specification("XL330-M288", c["voltage_v"])
    positive = ("torque_derating", "pwm_limit_fraction", "pid_scale", "target_speed_rad_s",
                "target_acceleration_rad_s2", "target_filter_hz", "total_mass_kg")
    nonnegative = ("joint_damping_nm_s_rad", "joint_friction_nm", "joint_armature_kg_m2",
                   "passive_damping_nm_s_rad", "passive_friction_nm")
    for key in positive + nonnegative:
        v = c[key]
        if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v < 0 or (key in positive and v == 0):
            raise ValueError("Invalid motor-model parameter " + key)
    if max(c["torque_derating"], c["pwm_limit_fraction"]) > 1:
        raise ValueError("Cannot increase datasheet torque or PWM")
    if c["mass_distribution"] not in ("uniform", "torso"):
        raise ValueError("Unknown mass distribution")
    if type(c["delay_frames"]) is not int or not 0 <= c["delay_frames"] <= 5:
        raise ValueError("Expected 0–5 whole 20 ms latency frames")


def modify_physics_xml(root, c):
    validate_motor_config(c)
    inertials = list(root.find("worldbody").iter("inertial"))
    total = sum(float(i.get("mass")) for i in inertials)
    if c["total_mass_kg"] < total:
        raise ValueError("Mass correction cannot remove CAD mass")
    for i in inertials:
        if c["mass_distribution"] == "uniform":
            factor = c["total_mass_kg"] / total
        elif i is root.find(".//body[@name='body_down_3dprint']/inertial"):
            factor = 1 + (c["total_mass_kg"] - total) / float(i.get("mass"))
        else:
            continue
        i.set("mass", str(float(i.get("mass")) * factor))
        attr = "fullinertia" if i.get("fullinertia") else "diaginertia"
        i.set(attr, " ".join(str(float(v) * factor) for v in i.get(attr).split()))
    for a in root.find("actuator"):
        j = root.find(".//joint[@name='%s']" % a.get("joint"))
        j.set("damping", str(c["joint_damping_nm_s_rad"]))
        j.set("frictionloss", str(c["joint_friction_nm"]))
        j.set("armature", str(c["joint_armature_kg_m2"]))
    # The CAD exporter assigned the motor class to passive ball joints too.
    # They have no geared rotor; do not copy a motor's reflected inertia there.
    for j in root.find("worldbody").iter("joint"):
        if j.get("type") == "ball":
            j.set("damping", str(c["passive_damping_nm_s_rad"]))
            j.set("frictionloss", str(c["passive_friction_nm"]))
            j.set("armature", "0")


def dc_torque(pwm, velocity, stall_torque, no_load_speed, cap):
    """Back EMF reduces driving torque; braking allowed, never beyond cap.

    No qvel clamp: external loads can back-drive the motor above no-load speed.
    Electrical inductance, temperature, supply sag and gearbox lash are omitted.
    """
    return np.clip(stall_torque * (pwm - velocity / no_load_speed), -cap, cap)


class DynamixelApproximation:
    def __init__(self, model, config):
        validate_motor_config(config)
        self.config, self.model = dict(config), model
        self.dt = model.opt.timestep
        self.qadr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
        self.vadr = model.jnt_dofadr[model.actuator_trnid[:, 0]]
        self.limits = model.actuator_ctrlrange.copy()
        self.names = [model.actuator(i).name for i in range(model.nu)]
        kinds = ["XC330-M288-proxy" if n == "yaw_body" else
                 "XL330-M288" if n.startswith("stewart") else "XL330-M077" for n in self.names]
        specs = [specification(k, config["voltage_v"]) for k in kinds]
        self.stall, self.speed = np.array(specs).T
        self.torque_caps = self.stall * config["torque_derating"]
        # Inferred count-error -> PWM mapping, with gain uncertainty in training.
        p = np.array([300 if n.startswith("stewart") else 200 for n in self.names])
        self.pwm_kp = p / 128 * (4096 / (2 * math.pi)) / 885 * config["pid_scale"]
        model.actuator_gainprm[:] = 0
        model.actuator_gainprm[:, 0] = 1
        model.actuator_biasprm[:] = 0
        model.actuator_ctrlrange[:] = np.column_stack((-self.torque_caps, self.torque_caps))
        model.actuator_forcerange[:] = model.actuator_ctrlrange

    def reset(self, targets):
        self.target = np.array(targets, dtype=float)
        self.filtered = self.target.copy()
        self.target_velocity = np.zeros_like(self.target)
        self.queue = deque([self.target.copy() for _ in range(self.config["delay_frames"])])
        self.steps = self.saturated_steps = 0
        self.max_command_speed = self.max_command_acceleration = 0.

    def command(self, targets):
        if not np.isfinite(targets).all() or np.any(targets < self.limits[:, 0]) or np.any(targets > self.limits[:, 1]):
            raise ValueError("Invalid position target")
        self.queue.append(targets.copy())
        self.target = self.queue.popleft()

    def apply(self, state):
        c = self.config
        speed_limit = np.minimum(c["target_speed_rad_s"], self.speed)
        desired_v = np.clip((self.target - self.filtered) * (2 * math.pi * c["target_filter_hz"]), -speed_limit, speed_limit)
        dv = np.clip(desired_v - self.target_velocity,
                     -c["target_acceleration_rad_s2"] * self.dt,
                     c["target_acceleration_rad_s2"] * self.dt)
        self.target_velocity += dv
        self.filtered = np.clip(self.filtered + self.dt * self.target_velocity, self.limits[:, 0], self.limits[:, 1])
        pwm_raw = self.pwm_kp * (self.filtered - state.qpos[self.qadr])
        pwm = np.clip(pwm_raw, -c["pwm_limit_fraction"], c["pwm_limit_fraction"])
        state.ctrl[:] = dc_torque(pwm, state.qvel[self.vadr], self.stall, self.speed, self.torque_caps)
        self.steps += 1
        self.saturated_steps += int(np.any(np.abs(pwm_raw) >= c["pwm_limit_fraction"]))
        self.max_command_speed = max(self.max_command_speed, float(np.max(np.abs(self.target_velocity))))
        self.max_command_acceleration = max(self.max_command_acceleration, float(np.max(np.abs(dv / self.dt))))

    def report(self):
        return {"model": "unidentified_PWM_DC_approximation", "hardware_ready": False,
                "stall_torque_nm": self.stall.tolist(), "torque_caps_nm": self.torque_caps.tolist(),
                "no_load_speed_rad_s": self.speed.tolist(),
                "any_pwm_saturated_fraction_including_warmup": self.saturated_steps / max(1, self.steps),
                "max_filtered_command_speed_rad_s": self.max_command_speed,
                "max_filtered_command_acceleration_rad_s2": self.max_command_acceleration,
                "thermal_model": False, "current_estimate": "not_calibrated_not_reported"}
