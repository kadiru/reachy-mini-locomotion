"""First-session host orchestration. Hardware writes go only through guardian."""
import json
import math
import socket
import threading
import time
import numpy as np
from .profile import QuestMapper
from .protocol import QuestGate, encode, decode
from .robot_io import request_json, STATE_PATH


class PreviewSink:
    """No robot connection. Acknowledgments are explicitly synthetic."""
    def __init__(self):
        self.lease = None
        self.synced_at = time.monotonic_ns()

    def arm(self):
        self.lease = "synthetic-preview"

    def sync(self):
        self.synced_at = time.monotonic_ns()

    def submit(self, target, valid_until_ns=None):
        if not self.lease:
            raise ValueError("Preview not armed")
        return {"accepted": True, "host_submission_ns": time.monotonic_ns(),
                "clock_uncertainty_ns": 0, "synthetic": True}

    def stop(self, reason):
        self.lease = None
        return {"mode": "STOPPED", "synthetic": True}


class RecordingSession:
    def __init__(self, recorder, profile, key, sink, mode="preview", require_camera=True):
        self.recorder, self.profile, self.sink, self.mode = recorder, profile, sink, mode
        self.gate = QuestGate(key, max_age_s=min(.2, profile.data["command_timeout_s"]))
        self.mapper = QuestMapper(profile)
        self.lock = threading.RLock()
        self.latest = None
        self.pose = None
        self.pose_ns = 0
        self.require_camera = require_camera
        self.status = "DISARMED"
        self.reason = None
        self.release_seen = False
        self.last_sequence = -1
        self.last_control_ns = time.monotonic_ns()
        self.start_ns = None
        self.start_xy = None
        self.stop_requested = threading.Event()
        self.quit = threading.Event()
        self.cycle = None
        self.cycle_number = 0

    def event(self, name, **extra):
        return self.recorder.write("event", dict(name=name, **extra), "session")

    def receive(self, raw):
        with self.lock:
            data = self.gate.accept(raw)
            if self.mode != "preview" and data.get("synthetic") is True:
                self.request_stop("synthetic_input_on_physical_session")
                raise ValueError("Synthetic input is only accepted in preview mode")
            receipt = time.monotonic_ns()
            self.recorder.write("quest_input", {"payload": data["packet"], "tracking_valid": data["tracking_valid"],
                "sender_id": data["sender_id"], "synthetic": data.get("synthetic", False), "source_capture_s": data["source_time_s"],
                "source_clock": "unity_monotonic_unmapped", "sample_t_ns": receipt-self.recorder.origin,
                "freshness_bound_s": self.gate.max_age, "mapping_to_sdk": "reconstructed_clutched_rpy_v1"}, "quest_udp")
            self.latest = (data, receipt)
            if data["packet"]["funny_engaged"]:
                self.request_stop("quest_grip_stop")

    def camera_update(self, pose, receipt_ns):
        with self.lock:
            self.pose, self.pose_ns = pose, receipt_ns

    def check_camera(self, now):
        if not self.require_camera:
            return
        if self.pose is None or now-self.pose_ns > 200_000_000:
            raise ValueError("Camera track missing or older than 200 ms")
        if max(abs(self.pose["roll_rad"]), abs(self.pose["pitch_rad"])) > self.profile.data["max_base_tilt_rad"]:
            raise ValueError("Base tilt exceeds configured envelope")
        xy = np.array([self.pose["x_m"], self.pose["y_m"]])
        if self.start_xy is not None and np.linalg.norm(xy-self.start_xy) > self.profile.data["max_workspace_radius_m"]:
            raise ValueError("Base left configured trial radius")

    def arm(self):
        with self.lock:
            if self.mode == "record-only":
                raise ValueError("Record-only mode cannot arm")
            if self.status in {"WAIT_RELEASE", "READY", "ACTIVE"}:
                raise ValueError("Already armed; stop before rearming")
            self.start_xy = None
            self.check_camera(time.monotonic_ns())
            self.gate.reset()
            self.latest = None
            self.mapper = QuestMapper(self.profile)
            self.stop_requested.clear()
            self.release_seen = False
            self.last_sequence = -1
            self.status, self.reason = "WAIT_RELEASE", None
            self.event("arm_requested", note="release trigger, then hold B and trigger; mapping is reconstructed")

    def request_stop(self, reason="operator_stop"):
        # Nonblocking latch: keyboard and camera errors need not wait on RPC.
        self.reason = reason
        self.stop_requested.set()

    def stop(self):
        with self.lock:
            reason = self.reason or "operator_stop"
            self.status = "STOPPED"
            try:
                normal = reason in {"Deadman released", "operator_stop", "operator_quit", "session_end"}
                self.event("teleop_stopped" if normal else "abort", reason=reason)
            finally:
                result = self.sink.stop(reason)
                self.event("stop_result", result=result, physical_settling_verified=False)
            self.latest = None

    def step(self):
        with self.lock:
            if self.stop_requested.is_set():
                if self.status != "STOPPED":
                    self.stop()
                return
            if self.mode == "record-only" or self.status not in {"WAIT_RELEASE", "READY", "ACTIVE"}:
                return
            now = time.monotonic_ns()
            try:
                self.check_camera(now)
                if self.latest is None:
                    return
                data, receipt = self.latest
                if now-receipt > self.gate.max_age*1e9:
                    raise ValueError("Quest input timeout")
                packet = data["packet"]
                if not data["tracking_valid"]:
                    raise ValueError("Quest tracking lost")
                if packet["a_temi_engaged"]:
                    raise ValueError("Temi selection conflicts with Mini control")
                engaged = packet["teleop_engaged"] and packet["b_reachy_engaged"]
                if self.status == "WAIT_RELEASE":
                    if not packet["teleop_engaged"]:
                        self.release_seen, self.status = True, "READY"
                    return
                if self.status == "READY":
                    if not engaged:
                        return
                    self.mapper.clutch(packet)
                    self.sink.arm()
                    now = time.monotonic_ns()
                    self.start_ns, self.last_control_ns = now, now
                    self.start_xy = None if self.pose is None else np.array([self.pose["x_m"], self.pose["y_m"]])
                    self.status = "ACTIVE"
                    self.event("teleop_started", controller_zero_rad=self.mapper.zero.tolist())
                    return
                if not engaged:
                    raise ValueError("Deadman released")
                if now-self.start_ns >= self.profile.data["max_trial_s"]*1e9:
                    raise ValueError("Trial time limit")
                if now-self.sink.synced_at > 1e9:
                    self.sink.sync()
                if packet["number"] == self.last_sequence:
                    return  # never refresh watchdog with repeated input
                dt = (now-self.last_control_ns)/1e9
                if dt > .1:
                    raise ValueError("Control loop stalled more than 100 ms")
                target, mapping = self.mapper.map(packet, dt)
                self.recorder.write("command", {"stage": "requested", "target": target,
                    "quest_sequence": packet["number"], **mapping}, "quest_bridge")
                if self.stop_requested.is_set():
                    self.stop()
                    return
                # Recheck after potentially blocking clock sync/logging. Bound
                # the remote deadline to the original observation ages too.
                before_submit = time.monotonic_ns()
                self.check_camera(before_submit)
                valid_until = receipt + int(self.gate.max_age*1e9)
                if self.require_camera:
                    valid_until = min(valid_until, self.pose_ns+200_000_000)
                if before_submit >= valid_until or before_submit-now > 100_000_000:
                    raise ValueError("Observation expired during target preparation")
                reply = self.sink.submit(target, valid_until_ns=valid_until)
                if reply.get("accepted") is not True:
                    raise ValueError("Guardian did not acknowledge command")
                self.recorder.write("command", {"stage": "submitted", "target": target,
                    "quest_sequence": packet["number"], "sample_t_ns": reply["host_submission_ns"]-self.recorder.origin,
                    "clock_uncertainty_ns": reply["clock_uncertainty_ns"], "synthetic": reply.get("synthetic", False),
                    **mapping}, "quest_bridge")
                if "state" in reply:
                    self.recorder.write("state", {"sdk_state": reply["state"],
                        "sample_time_basis": "guardian_pre_submission_received_at_host"}, "guardian_response")
                self.last_control_ns, self.last_sequence = now, packet["number"]
            except Exception as error:
                self.request_stop(str(error))
                self.stop()

    def key(self, key):
        if key in (" ", "x", "\x1b"):
            self.request_stop("operator_stop")
        elif key == "q":
            self.request_stop("operator_quit")
            self.quit.set()
        elif key == "a":
            self.arm()
        elif key in ("f", "h"):
            self.event("fall" if key == "f" else "external_assistance")
            self.request_stop("operator_incident")
        elif key == "c" and self.cycle is None:
            self.cycle_number += 1
            self.cycle = str(self.cycle_number)
            self.event("cycle_start", cycle_id=self.cycle)
        elif key == "v" and self.cycle is not None:
            self.event("cycle_end", cycle_id=self.cycle)
            self.cycle = None
        elif key == "m":
            self.event("sync_marker", method="operator_key", visible_flash_verified=False)


def udp_worker(session, key, address, ready=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(address)
    sock.settimeout(.1)
    if ready:
        ready(sock.getsockname())
    last_error = 0.
    try:
        while not session.quit.is_set():
            try:
                raw, peer = sock.recvfrom(8193)
                decode(raw, key)  # only respond to authenticated clients
                try:
                    session.receive(raw)
                finally:
                    with session.lock:
                        challenge = session.gate.challenge()
                    sock.sendto(encode(challenge, key), peer)
            except socket.timeout:
                continue
            except (ValueError, KeyError, TypeError, UnicodeError) as error:
                # Bad packets never refresh freshness. Rate-limit diagnostics.
                if time.monotonic()-last_error > 1:
                    session.event("quest_packet_rejected", detail=str(error))
                    last_error = time.monotonic()
    finally:
        sock.close()


def control_worker(session):
    try:
        while not session.quit.is_set():
            start = time.monotonic()
            session.step()
            session.quit.wait(max(0, .02-(time.monotonic()-start)))
    except Exception as error:
        session.request_stop("control_worker_failed: " + str(error))
        try:
            session.stop()
        finally:
            session.quit.set()


def state_worker(session, origin):
    while not session.quit.is_set():
        start = time.monotonic_ns()
        try:
            state = request_json(origin, STATE_PATH, timeout=.3)
            session.recorder.write("state", {"sdk_state": state, "request_t_ns": start-session.recorder.origin,
                "sample_time_basis": "host_receipt"}, "read_only_daemon")
        except Exception as error:
            session.event("state_read_error", detail=str(error))
        session.quit.wait(.05)


def align_recording(source, destination):
    """Sort explicitly known host sample times; retain raw log/provenance."""
    from .recording import load, validate
    rows = load(source)
    aligned = [rows[0]]
    for original in rows[1:]:
        row = dict(original, data=dict(original["data"]))
        sample = row["data"].get("sample_t_ns")
        if sample is not None:
            if type(sample) is not int or sample < 0:
                raise ValueError("Invalid mapped sample timestamp")
            row["data"]["raw_record_t_ns"] = row["t_ns"]
            row["t_ns"] = sample
        aligned.append(row)
    aligned = [aligned[0]] + sorted(aligned[1:], key=lambda row: row["t_ns"])
    aligned[0] = dict(aligned[0], alignment={"source": str(source), "method": "known_host_sample_times",
                      "camera_basis": "frame_receipt_not_exposure", "quest_source_clock_mapped": False})
    validate(aligned)
    with open(destination, "x") as f:
        for row in aligned:
            f.write(json.dumps(row, allow_nan=False)+"\n")
    return len(aligned)
