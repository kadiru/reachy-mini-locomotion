"""Measured hardware envelope and an explicitly reconstructed Quest mapping."""
import hashlib
import json
import math
import numpy as np
from .geometry import transform, rotation, euler, wrapped, angle
from .recording import finite


class Profile:
    def __init__(self, data, execute=False):
        self.data = data
        finite(data)
        if data.get("schema_version") != 1:
            raise ValueError("Profile schema_version must be 1")
        self.hash = hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.neutral = transform(data["neutral_head"])
        self.body = float(data["neutral_body_yaw_rad"])
        self.rotation_limits = self.vector("max_head_rpy_delta_rad", 3, positive=True)
        self.gains = self.vector("quest_rpy_gains", 3)
        self.joint_min = self.vector("joint_min_rad", 7)
        self.joint_max = self.vector("joint_max_rad", 7)
        if np.any(self.joint_min >= self.joint_max):
            raise ValueError("Joint limits must be ordered")
        for key in ("max_body_delta_rad", "max_head_speed_rad_s", "max_body_speed_rad_s",
                    "max_head_accel_rad_s2", "max_body_accel_rad_s2", "max_joint_speed_rad_s",
                    "max_tracking_error_rad", "start_tolerance_rad", "command_timeout_s", "max_trial_s",
                    "max_base_tilt_rad", "max_workspace_radius_m"):
            value = data[key]
            if type(value) not in (int, float) or value <= 0:
                raise ValueError("Positive finite profile value required: " + key)
        if data["command_timeout_s"] > .5 or data["max_trial_s"] > 60:
            raise ValueError("First-session timeout <=0.5 s and trial <=60 s")
        if type(data.get("automatic_body_yaw")) is not bool:
            raise ValueError("Specify installed automatic_body_yaw convention")
        if type(data.get("quest_body_yaw_gain")) not in (int, float):
            raise ValueError("Numeric Quest body yaw gain required")
        if execute:
            for field in ("hardware_verified", "exclusive_motor_owner_verified", "stop_hold_verified"):
                if data.get(field) is not True:
                    raise ValueError("Hardware profile not ready: " + field)
            for field in ("robot_id", "installed_sdk_version", "verification_notes"):
                if not isinstance(data.get(field), str) or not data[field].strip() or "REPLACE" in data[field]:
                    raise ValueError("Fill measured profile field: " + field)

    def vector(self, key, length, positive=False):
        a = np.asarray(self.data[key], dtype=float)
        if a.shape != (length,) or not np.all(np.isfinite(a)) or (positive and np.any(a <= 0)):
            raise ValueError("Invalid vector: " + key)
        return a

    def check_target(self, target):
        if set(target) != {"head", "body_yaw"}:
            raise ValueError("First-session target must contain only head and body_yaw")
        head = transform(target["head"])
        body = target["body_yaw"]
        if type(body) not in (int, float) or not math.isfinite(body):
            raise ValueError("Invalid body yaw")
        delta = np.linalg.inv(self.neutral) @ head
        if np.linalg.norm(delta[:3, 3]) > 1e-7:
            raise ValueError("Translation is locked for initial reconstructed mapping")
        if np.any(np.abs(euler(delta)) > self.rotation_limits + 1e-8):
            raise ValueError("Target outside head envelope")
        if abs(body-self.body) > self.data["max_body_delta_rad"] + 1e-8:
            raise ValueError("Target outside body envelope")
        return head, body

    def check_joints(self, values):
        q = np.asarray(values, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)) or np.any(q < self.joint_min) or np.any(q > self.joint_max):
            raise ValueError("Invalid/out-of-envelope joint state or IK")
        return q


class QuestMapper:
    """Clutched neutral, degree conversion, joint-independent task-space targets.

    This is a new mapping; the original deployed mapping has not been recovered.
    Input limiting is logged. Guardian separately rejects invalid final targets.
    """
    def __init__(self, profile):
        self.profile = profile
        self.zero = None
        self.rpy = np.zeros(3)
        self.body_delta = 0.
        self.velocity = np.zeros(3)
        self.body_velocity = 0.

    @staticmethod
    def orientation(packet):
        return np.deg2rad([packet["roll_deg"], packet["pitch_deg"], packet["yaw_deg"]])

    def clutch(self, packet):
        self.zero = self.orientation(packet)
        self.rpy[:] = 0
        self.velocity[:] = 0
        self.body_delta = self.body_velocity = 0.

    def map(self, packet, dt):
        if self.zero is None or not 0 < dt <= .1:
            raise ValueError("Clutch and a fresh update interval required")
        p = self.profile
        delta = np.array([wrapped(x) for x in self.orientation(packet)-self.zero])
        requested = delta * p.gains
        desired = np.clip(requested, -p.rotation_limits, p.rotation_limits)
        v = (desired-self.rpy)/dt
        speed = np.linalg.norm(v)
        # Euler-axis rates bound SO(3) angular rate by their L1 norm. Retain
        # margin for independent guardian timing and discretization.
        rate_limit = .8*p.data["max_head_speed_rad_s"]/math.sqrt(3)
        if speed > rate_limit:
            v *= rate_limit/speed
        dv = v-self.velocity
        cap = p.data["max_head_accel_rad_s2"]*dt
        if np.linalg.norm(dv) > cap:
            dv *= cap/np.linalg.norm(dv)
        self.velocity += dv
        self.rpy = np.clip(self.rpy + self.velocity*dt, -p.rotation_limits, p.rotation_limits)
        body_req = delta[2]*p.data["quest_body_yaw_gain"]
        body_desired = float(np.clip(body_req, -p.data["max_body_delta_rad"], p.data["max_body_delta_rad"]))
        body_v = float(np.clip((body_desired-self.body_delta)/dt, -.8*p.data["max_body_speed_rad_s"], .8*p.data["max_body_speed_rad_s"]))
        self.body_velocity += float(np.clip(body_v-self.body_velocity, -p.data["max_body_accel_rad_s2"]*dt, p.data["max_body_accel_rad_s2"]*dt))
        self.body_delta = float(np.clip(self.body_delta+self.body_velocity*dt, -p.data["max_body_delta_rad"], p.data["max_body_delta_rad"]))
        head = p.neutral.copy()
        head[:3, :3] = p.neutral[:3, :3] @ rotation(self.rpy)
        target = {"head": head.tolist(), "body_yaw": p.body+self.body_delta}
        p.check_target(target)
        return target, {"mapping": "reconstructed_clutched_rpy_v1", "requested_rpy_rad": requested.tolist(),
                        "limited_rpy_rad": self.rpy.tolist(), "profile_sha256": p.hash}
