"""Robot-local single writer, expiring lease, latched stop, and audit journal.

Software stop submits the measured pose once; it does not disable torque or
claim a certified emergency stop. Must run on Mini for host-loss protection.
"""
import hmac
import json
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from .geometry import transform, angle
from .robot_io import state_target, request_json


class Guardian:
    def __init__(self, robot, profile, recorder, clock=time.monotonic_ns):
        self.robot, self.profile, self.recorder, self.clock = robot, profile, recorder, clock
        self.lock = threading.RLock()
        self.lease = None
        self.status = "DISARMED"
        self.last_target = None
        self.last_q = None
        self.last_ns = 0
        self.expires_ns = 0
        self.started_ns = 0
        self.sequence = -1
        self.reason = None

    def event(self, name, **extra):
        self.recorder.write("event", dict(name=name, **extra), "guardian")

    def snapshot(self):
        with self.lock:
            return {"mode": self.status, "reason": self.reason, "profile_sha256": self.profile.hash,
                    "evidence": self.robot.evidence, "monotonic_ns": self.clock()}

    def read_state(self):
        start = self.clock()
        state = self.robot.state()
        self.recorder.write("state", {"sdk_state": state, "request_t_ns": start-self.recorder.origin,
                            "sample_time_basis": "host_receipt"}, "guardian_daemon")
        return state

    def validate_state(self, state):
        if state.get("control_mode") != "enabled":
            raise ValueError("Motors must already be enabled by the normal robot setup")
        q = self.profile.check_joints(state["head_joints"])
        target_q = self.profile.check_joints(state["target_head_joints"])
        if np.max(np.abs(q-target_q)) > self.profile.data["max_tracking_error_rad"]:
            raise ValueError("Joint tracking error outside profile")
        return q

    def arm(self, profile_hash):
        with self.lock:
            if self.lease is not None:
                raise ValueError("Another active lease owns the robot")
            if profile_hash != self.profile.hash:
                raise ValueError("Bridge and guardian profiles differ")
            state = self.robot.preflight()
            q = self.validate_state(state)
            actual = state_target(state)
            if angle(transform(actual["head"]), self.profile.neutral) > self.profile.data["start_tolerance_rad"]:
                raise ValueError("Head must be at the measured neutral pose before arming")
            if np.linalg.norm(transform(actual["head"])[:3, 3]-self.profile.neutral[:3, 3]) > .002:
                raise ValueError("Head translation differs from neutral by more than 2 mm")
            if abs(actual["body_yaw"]-self.profile.body) > self.profile.data["start_tolerance_rad"]:
                raise ValueError("Body must be at measured neutral before arming")
            self.last_target, self.last_q = actual, q
            self.started_ns = self.last_ns = self.clock()
            self.expires_ns = self.last_ns + int(self.profile.data["command_timeout_s"]*1e9)
            self.sequence = -1
            self.lease = secrets.token_hex(24)
            self.status, self.reason = "ARMED", None
            self.event("guardian_armed", profile_sha256=profile_hash)
            return {"lease": self.lease, "monotonic_ns": self.clock()}

    def stop(self, reason="operator_stop"):
        with self.lock:
            had_lease = self.lease is not None
            self.lease = None  # invalidate before any blocking I/O
            self.status, self.reason = "STOPPED", reason
            if not had_lease:
                return self.snapshot()
            try:
                self.event("stop_requested", reason=reason)
                state = self.read_state()
                self.profile.check_joints(state["head_joints"])
                hold = state_target(state)
                self.recorder.write("command", {"stage": "requested", "target": hold, "purpose": "stop_hold"}, "guardian")
                self.robot.submit(hold)
                self.recorder.write("command", {"stage": "submitted", "target": hold, "purpose": "stop_hold"}, "guardian")
                self.event("stop_hold_submitted", reason=reason, physical_settling_verified=False)
            except Exception as error:
                self.status = "FAULT"
                self.reason = reason + "; hold failed: " + str(error)
                # If disk logging itself failed, stopping must still be attempted.
                try:
                    state = self.robot.state()
                    self.robot.submit(state_target(state))
                except Exception:
                    pass
                try:
                    self.event("stop_hold_failed", detail=self.reason)
                except Exception:
                    pass
            return self.snapshot()

    def tick(self):
        with self.lock:
            now = self.clock()
            if self.lease and (now >= self.expires_ns or now-self.started_ns >= self.profile.data["max_trial_s"]*1e9):
                self.stop("watchdog_timeout")

    def command(self, body):
        with self.lock:
            self.tick()
            if not self.lease or not hmac.compare_digest(str(body.get("lease", "")), self.lease):
                raise ValueError("No matching active lease")
            try:
                now = self.clock()
                seq, deadline = body["sequence"], body["deadline_ns"]
                if type(seq) is not int or seq <= self.sequence:
                    raise ValueError("Duplicate/reordered target")
                if type(deadline) is not int or not now < deadline <= now + int(self.profile.data["command_timeout_s"]*1e9):
                    raise ValueError("Expired or excessive command deadline")
                head, yaw = self.profile.check_target(body["target"])
                q = self.profile.check_joints(self.robot.ik(body["target"]))
                state = self.read_state()
                self.validate_state(state)
                # Detect another target writer. This cannot lock out other SDK
                # apps globally; exclusive ownership is an explicit preflight.
                previous = np.asarray(state["target_head_pose"]["m"]).reshape(4, 4)
                expected = transform(self.last_target["head"])
                target_body = state.get("target_body_yaw", state["body_yaw"])
                if (angle(previous, expected) > .01 or np.linalg.norm(previous[:3, 3]-expected[:3, 3]) > .001
                        or abs(target_body-self.last_target["body_yaw"]) > .01):
                    raise ValueError("Daemon head target changed outside this writer")
                dt = min((self.clock()-self.last_ns)/1e9, .1)
                if dt <= 0:
                    raise ValueError("Nonpositive command interval")
                if angle(head, transform(self.last_target["head"])) > self.profile.data["max_head_speed_rad_s"]*dt + 1e-6:
                    raise ValueError("Head target rate exceeded")
                if abs(yaw-self.last_target["body_yaw"]) > self.profile.data["max_body_speed_rad_s"]*dt + 1e-6:
                    raise ValueError("Body target rate exceeded")
                if np.max(np.abs(q-self.last_q)) > self.profile.data["max_joint_speed_rad_s"]*dt + 1e-6:
                    raise ValueError("IK joint rate exceeded")
                if self.clock() >= min(deadline, self.expires_ns):
                    raise ValueError("Target expired during validation")
                self.recorder.write("command", {"stage": "requested", "target": body["target"], "sequence": seq}, "guardian")
                self.robot.submit(body["target"])
                submitted = self.clock()
                self.recorder.write("command", {"stage": "submitted", "target": body["target"], "sequence": seq,
                                    "submission_monotonic_ns": submitted}, "guardian")
                self.last_target, self.last_q, self.last_ns = body["target"], q, submitted
                self.sequence, self.expires_ns = seq, deadline
                self.status = "ACTIVE"
                return {"accepted": True, "sequence": seq, "submitted_ns": submitted,
                        "state": state, "state_time_basis": "pre_submission_host_receipt",
                        "synthetic": self.robot.evidence == "synthetic"}
            except Exception as error:
                self.stop("command_rejected: " + str(error))
                raise


def make_server(guardian, key, address):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # never log request credentials

        def handle_rpc(self):
            t1 = guardian.clock()
            if not hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer " + key.decode()):
                self.send_error(401)
                return
            try:
                body = {}
                if self.command == "POST":
                    n = int(self.headers.get("Content-Length", "0"))
                    if not 0 < n <= 8192:
                        raise ValueError("Bounded JSON body required")
                    self.connection.settimeout(.3)
                    body = json.loads(self.rfile.read(n))
                if self.path == "/clock" and self.command == "GET":
                    result = {"receive_ns": t1, "send_ns": guardian.clock()}
                elif self.path == "/status" and self.command == "GET":
                    result = guardian.snapshot()
                elif self.path == "/state" and self.command == "GET":
                    with guardian.lock:
                        result = guardian.read_state()
                elif self.path == "/arm" and self.command == "POST":
                    result = guardian.arm(body["profile_sha256"])
                elif self.path == "/command" and self.command == "POST":
                    result = guardian.command(body)
                elif self.path == "/stop" and self.command == "POST":
                    result = guardian.stop(str(body.get("reason", "operator_stop"))[:200])
                else:
                    raise ValueError("Unknown RPC")
                status = 200
            except Exception as error:
                result, status = {"error": str(error)}, 409
            raw = json.dumps(result, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        do_GET = do_POST = handle_rpc

    return ThreadingHTTPServer(address, Handler)


class GuardianClient:
    def __init__(self, origin, key, profile, recorder=None):
        from urllib.parse import urlparse
        u = urlparse(origin)
        if u.scheme != "http" or not u.hostname or u.path not in ("", "/") or u.username or u.password or u.query or u.fragment:
            raise ValueError("Expected explicit guardian HTTP origin on trusted LAN")
        self.origin, self.key, self.profile, self.recorder = origin.rstrip("/"), key, profile, recorder
        self.lease = None
        self.offset = self.uncertainty = None
        self.synced_at = 0
        self.sequence = 0

    def rpc(self, path, data=None):
        return request_json(self.origin, path, data, self.key, timeout=.3)

    def sync(self):
        t0 = time.monotonic_ns()
        remote = self.rpc("/clock")
        t3 = time.monotonic_ns()
        t1, t2 = remote["receive_ns"], remote["send_ns"]
        offset = ((t1-t0)+(t2-t3))/2
        uncertainty = ((t3-t0)-(t2-t1))/2
        if uncertainty < 0 or uncertainty > 25_000_000:
            raise ValueError("Guardian clock exchange exceeds 25 ms uncertainty")
        self.offset, self.uncertainty, self.synced_at = offset, uncertainty, t3
        if self.recorder:
            self.recorder.write("event", {"name": "clock_mapping", "remote": "guardian", "t0_ns": t0,
                "t1_ns": t1, "t2_ns": t2, "t3_ns": t3, "offset_ns": offset,
                "uncertainty_ns": uncertainty, "drift_allowance_ns_per_s": 1_000_000}, "bridge")

    def arm(self):
        self.sync()
        response = self.rpc("/arm", {"profile_sha256": self.profile.hash})
        self.lease = response["lease"]
        self.sequence = 0

    def submit(self, target, valid_until_ns=None):
        now = time.monotonic_ns()
        if not self.lease or now-self.synced_at > 2e9:
            raise ValueError("Active lease and recent clock mapping required")
        self.sequence += 1
        uncertainty = self.uncertainty + (now-self.synced_at)*.001
        local_deadline = now + self.profile.data["command_timeout_s"]*1e9
        if valid_until_ns is not None:
            local_deadline = min(local_deadline, valid_until_ns)
        deadline = int(local_deadline + self.offset - uncertainty)
        response = self.rpc("/command", {"lease": self.lease, "sequence": self.sequence,
                             "deadline_ns": deadline, "target": target})
        response["host_submission_ns"] = int(response["submitted_ns"]-self.offset)
        response["clock_uncertainty_ns"] = uncertainty
        return response

    def stop(self, reason="operator_stop"):
        self.lease = None
        return self.rpc("/stop", {"reason": reason})
