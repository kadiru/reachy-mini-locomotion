"""Version 1 JSONL format. All event times use one recorder's monotonic clock."""
import json
import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

KINDS = {"quest_input", "command", "state", "base_pose", "video_frame", "event"}


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("NaN/Infinity are not measurements")
    if isinstance(value, dict):
        for item in value.values():
            finite(item)
    if isinstance(value, list):
        for item in value:
            finite(item)


def validate(rows):
    if not rows or rows[0].get("kind") != "session":
        raise ValueError("First row must be session metadata")
    meta = rows[0]
    if meta.get("schema_version") != 1:
        raise ValueError("Unsupported schema version")
    if meta.get("evidence") not in {"physical", "simulated", "synthetic"}:
        raise ValueError("Explicit evidence class required")
    if meta.get("control_mode") not in {"teleoperation", "replay", "parameter_optimization", "learned_policy"}:
        raise ValueError("Explicit control mode required")
    for field in ("session_id", "robot", "setup", "clock", "measurement"):
        if field not in meta:
            raise ValueError("Missing metadata: " + field)
    previous = -1
    for row in rows[1:]:
        if row.get("kind") not in KINDS:
            raise ValueError("Unknown record kind")
        t = row.get("t_ns")
        if type(t) is not int or t < 0 or t < previous:
            raise ValueError("t_ns must be nonnegative, ordered integer nanoseconds")
        previous = t
        if not row.get("source") or not isinstance(row.get("data"), dict):
            raise ValueError("source and data required")
        data = row["data"]
        if row["kind"] == "base_pose":
            if data.get("frame") != "world" or data.get("tracked_body") != "foot":
                raise ValueError("Displacement requires world-frame foot pose, not head pose")
            for key in ("x_m", "y_m", "yaw_rad"):
                if type(data.get(key)) not in (int, float):
                    raise ValueError("Missing numeric base pose: " + key)
        if row["kind"] == "command":
            validate_target(data)
        if row["kind"] == "event" and not data.get("name"):
            raise ValueError("Event name required")
    finite(rows)
    return rows


def validate_target(data):
    if data.get("stage") not in {"requested", "submitted"}:
        raise ValueError("Command stage must distinguish request from submission")
    target = data.get("target", {})
    if not target or set(target) - {"head", "antennas", "body_yaw"}:
        raise ValueError("Expected SDK set_target fields")
    if "head" in target:
        h = target["head"]
        if not isinstance(h, list) or len(h) != 4 or any(not isinstance(r, list) or len(r) != 4 for r in h):
            raise ValueError("Head target must be a 4x4 matrix")
        if any(type(v) not in (float, int) for r in h for v in r):
            raise ValueError("Non-numeric head target")
        if any(abs(a - b) > 1e-6 for a, b in zip(h[3], [0, 0, 0, 1])):
            raise ValueError("Invalid homogeneous transform")
        r = [row[:3] for row in h[:3]]
        for i in range(3):
            for j in range(3):
                if abs(sum(r[k][i] * r[k][j] for k in range(3)) - (i == j)) > 1e-5:
                    raise ValueError("Head rotation must be orthonormal")
        det = sum(r[0][i] * (r[1][(i+1)%3] * r[2][(i+2)%3] - r[1][(i+2)%3] * r[2][(i+1)%3]) for i in range(3))
        if abs(det - 1) > 1e-5:
            raise ValueError("Head rotation must be right-handed")
    if "antennas" in target and (not isinstance(target["antennas"], list) or len(target["antennas"]) != 2 or any(type(v) not in (float, int) for v in target["antennas"])):
        raise ValueError("Antennas must be [right, left] radians")
    if "body_yaw" in target and type(target["body_yaw"]) not in (float, int):
        raise ValueError("Body yaw must be radians")
    finite(data)


def load(path):
    with open(path) as f:
        return validate([json.loads(line) for line in f if line.strip()])


class Recorder:
    """Thread-safe capture hook, same instance for command and state logging.

    Receipt timestamps are not sensor exposure timestamps. Preserve source stamps
    in data and establish clock mappings before making latency/sync claims.
    """
    def __init__(self, path, metadata):
        self.origin = time.monotonic_ns()
        self.lock = threading.Lock()
        self.meta = dict(metadata, schema_version=1, kind="session", clock={
            "type": "host_monotonic", "origin_monotonic_ns": self.origin,
            "started_utc": datetime.now(timezone.utc).isoformat(),
        })
        validate([self.meta])
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.file = open(path, "x")
        self.file.write(json.dumps(self.meta, allow_nan=False) + "\n")
        self.file.flush()

    def write(self, kind, data, source):
        with self.lock:
            row = {"kind": kind, "t_ns": time.monotonic_ns() - self.origin,
                   "source": source, "data": data}
            validate([self.meta, row])
            self.file.write(json.dumps(row, allow_nan=False) + "\n")
            self.file.flush()
        return row

    def close(self):
        with self.lock:
            self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
