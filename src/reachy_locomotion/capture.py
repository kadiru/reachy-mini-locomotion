"""Emcee packet capture and optional read-only Reachy HTTP sampling."""
import json
import time
import urllib.request
from urllib.parse import urlparse
from .recording import finite


def parse_emcee(payload):
    data = json.loads(payload) if isinstance(payload, (str, bytes)) else dict(payload)
    fields = {"number", "teleop_engaged", "a_temi_engaged", "b_reachy_engaged", "funny_engaged", "pitch_deg", "roll_deg", "yaw_deg", "stick_x", "stick_y"}
    if set(data) != fields:
        raise ValueError("Unexpected Emcee fields; preserve raw packet separately and inspect")
    finite(data)
    if type(data["number"]) is not int or data["number"] < 0:
        raise ValueError("Invalid sequence number")
    for key in fields - {"number"}:
        if key.endswith("engaged"):
            if type(data[key]) is not bool:
                raise ValueError("Invalid button " + key)
        elif type(data[key]) not in (int, float):
            raise ValueError("Invalid numeric field " + key)
    if any(abs(data[k]) > 1 for k in ("stick_x", "stick_y")):
        raise ValueError("Stick outside [-1, 1]")
    return data


def capture_emcee(recorder, payload, uid, stream_id):
    """Call in Emcee onStreamMessage AFTER decoding. No robot command is issued."""
    return recorder.write("quest_input", {
        "payload": parse_emcee(payload), "sender_uid": uid, "stream_id": stream_id,
        "source_capture_ns": None, "mapping_to_sdk": "unknown",
    }, "emcee_agora_receive")


def poll_state(recorder, base_url, duration_s=10, hz=20):
    """GET only; no discovery, waking, motor enable, or target writes.

    Sampling speed is a request, not a guaranteed fresh sensor rate. Records
    request/receipt times and daemon timestamp; keep network delay visible.
    """
    u = urlparse(base_url)
    if u.scheme not in {"http", "https"} or not u.hostname or u.username or u.password or u.query or u.fragment or u.path not in {"", "/"}:
        raise ValueError("Expected explicit daemon origin, e.g. http://192.168.1.20:8000")
    if not 0 < hz <= 50 or not 0 < duration_s <= 300:
        raise ValueError("Use 0 < hz <= 50 and 0 < duration <= 300 seconds")
    url = base_url.rstrip("/") + "/api/state/full?with_head_joints=true&with_target_head_joints=true&with_target_head_pose=true&with_target_body_yaw=true&with_target_antenna_positions=true&with_imu=true&use_pose_matrix=true"
    start = time.monotonic()
    samples, errors = 0, 0
    while time.monotonic() - start < duration_s:
        requested = time.monotonic_ns() - recorder.origin
        tick = time.monotonic()
        try:
            with urllib.request.urlopen(url, timeout=min(2, duration_s)) as response:
                state = json.load(response)
            if not isinstance(state, dict):
                raise ValueError("Expected state object")
            recorder.write("state", {"request_t_ns": requested, "sample_time_basis": "host_receipt", "sdk_state": state}, "reachy_rest_full")
            samples += 1
        except Exception as error:
            recorder.write("event", {"name": "state_read_error", "detail": str(error)}, "reachy_rest_full")
            errors += 1
        time.sleep(max(0, 1/hz - (time.monotonic()-tick)))
    return {"state_samples": samples, "read_errors": errors, "elapsed_s": time.monotonic()-start}
