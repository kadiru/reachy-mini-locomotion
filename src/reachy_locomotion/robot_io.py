"""Bounded REST calls to the audited daemon API; no motor enable or waking."""
import copy
import importlib.metadata
import json
import time
import urllib.request
import numpy as np
from .geometry import transform

STATE_PATH = "/api/state/full?with_head_joints=true&with_target_head_joints=true&with_target_head_pose=true&with_target_body_yaw=true&with_imu=true&use_pose_matrix=true"


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError("Redirects are not permitted for robot RPC")


def request_json(origin, path, body=None, token=None, timeout=.2):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = "Bearer " + token.decode("ascii")
    data = None if body is None else json.dumps(body, allow_nan=False).encode()
    req = urllib.request.Request(origin+path, data=data, headers=headers)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    with opener.open(req, timeout=timeout) as response:
        raw = response.read(1_000_001)
    if len(raw) > 1_000_000:
        raise ValueError("RPC response too large")
    result = json.loads(raw)
    return result


def state_target(state):
    raw = state["head_pose"]
    head = np.asarray(raw["m"], dtype=float).reshape(4, 4)
    transform(head)
    yaw = state["body_yaw"]
    if type(yaw) not in (int, float) or not np.isfinite(yaw):
        raise ValueError("Invalid measured body yaw")
    return {"head": head.tolist(), "body_yaw": yaw}


class MockRobot:
    """Deterministic actuator stand-in, not a simulator or balance model."""
    evidence = "synthetic"

    def __init__(self, profile):
        self.profile = profile
        self.target = {"head": profile.neutral.tolist(), "body_yaw": profile.body}
        self.submissions = []

    def ik(self, target):
        return np.zeros(7) + target["body_yaw"]

    def state(self):
        return {"head_pose": {"m": np.asarray(self.target["head"]).ravel().tolist()},
                "target_head_pose": {"m": np.asarray(self.target["head"]).ravel().tolist()},
                "head_joints": self.ik(self.target).tolist(), "target_head_joints": self.ik(self.target).tolist(),
                "body_yaw": self.target["body_yaw"], "control_mode": "enabled", "imu": None,
                "synthetic": True}

    def preflight(self):
        return self.state()

    def submit(self, target):
        self.target = copy.deepcopy(target)
        self.submissions.append(copy.deepcopy(target))
        return {"status": "ok", "synthetic": True}


class LocalReachy:
    evidence = "physical"

    def __init__(self, profile, port=8000):
        if importlib.metadata.version("reachy-mini") != profile.data["installed_sdk_version"]:
            raise ValueError("Installed Reachy SDK version differs from measured hardware profile")
        from reachy_mini.kinematics import AnalyticalKinematics
        self.kinematics = AnalyticalKinematics(automatic_body_yaw=profile.data["automatic_body_yaw"])
        self.origin = "http://127.0.0.1:" + str(port)
        self.timeout = min(.15, profile.data["command_timeout_s"]/2)

    def state(self):
        return request_json(self.origin, STATE_PATH, timeout=self.timeout)

    def preflight(self):
        if request_json(self.origin, "/api/move/running", timeout=self.timeout):
            raise ValueError("Daemon has a running move")
        info = request_json(self.origin, "/api/kinematics/info", timeout=self.timeout)
        if info.get("info", {}).get("engine") != "AnalyticalKinematics":
            # Accept only the audited engine; installation-specific names must
            # be inspected rather than silently guessing compatible semantics.
            raise ValueError("Expected audited AnalyticalKinematics engine; inspect /api/kinematics/info")
        return self.state()

    def ik(self, target):
        return self.kinematics.ik(np.asarray(target["head"]), body_yaw=target["body_yaw"])

    def submit(self, target):
        payload = {"target_head_pose": {"m": np.asarray(target["head"]).ravel().tolist()},
                   "target_body_yaw": target["body_yaw"]}
        result = request_json(self.origin, "/api/move/set_target", payload, timeout=self.timeout)
        if result.get("status") != "ok":
            raise RuntimeError("Daemon did not accept target: " + str(result))
        return result
