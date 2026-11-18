"""Cycle displacement with bounded interpolation and explicit missing evidence."""
import bisect
import math
import statistics
from .recording import validate


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def analyze(rows):
    validate(rows)
    meta = rows[0]
    poses = [r for r in rows[1:] if r["kind"] == "base_pose"]
    times = [r["t_ns"] for r in poses]
    if len(set(times)) != len(times):
        raise ValueError("Duplicate base pose timestamps")
    max_gap = meta["measurement"].get("max_pose_gap_s", 0.1)
    if type(max_gap) not in (int, float) or max_gap <= 0:
        raise ValueError("Positive max_pose_gap_s required")
    max_gap_ns = max_gap * 1e9

    def at(t):
        i = bisect.bisect_left(times, t)
        if i < len(times) and times[i] == t:
            return poses[i]["data"]
        if i == 0 or i == len(times) or times[i] - times[i-1] > max_gap_ns:
            raise ValueError("missing or distant base-pose boundary")
        a, b = poses[i-1]["data"], poses[i]["data"]
        f = (t-times[i-1]) / (times[i]-times[i-1])
        return {"x_m": a["x_m"] + f*(b["x_m"]-a["x_m"]),
                "y_m": a["y_m"] + f*(b["y_m"]-a["y_m"]),
                "yaw_rad": wrap(a["yaw_rad"] + f*wrap(b["yaw_rad"]-a["yaw_rad"]))}

    events = [r for r in rows[1:] if r["kind"] == "event"]
    starts, ends = {}, {}
    for e in events:
        d = e["data"]
        if d["name"] in {"cycle_start", "cycle_end"}:
            key = d.get("cycle_id")
            target = starts if d["name"] == "cycle_start" else ends
            if not isinstance(key, str) or key in target:
                raise ValueError("Cycle IDs must be strings with unique start/end")
            target[key] = e["t_ns"]
    if set(ends) - set(starts):
        raise ValueError("Cycle end without start")
    complete = sorted((t, ends[k]) for k, t in starts.items() if k in ends)
    if any(a >= b for a, b in complete) or any(complete[i][0] < complete[i-1][1] for i in range(1, len(complete))):
        raise ValueError("Cycles must have positive duration and cannot overlap")
    cycles = []
    for key, start in starts.items():
        end = ends.get(key)
        c = {"cycle_id": key, "valid_displacement": False}
        if end is None:
            c["reason"] = "incomplete cycle"
            cycles.append(c)
            continue
        c["duration_s"] = (end-start)/1e9
        # Events at a shared boundary belong to the cycle ending there.
        in_cycle = [e["data"]["name"] for e in events if start < e["t_ns"] <= end]
        c["fall_events"] = in_cycle.count("fall")
        c["aborted"] = "abort" in in_cycle
        c["external_assistance"] = "external_assistance" in in_cycle
        try:
            a, b = at(start), at(end)
            inside = [p["t_ns"] for p in poses if start < p["t_ns"] < end]
            coverage = [start] + inside + [end]
            if any(v-u > max_gap_ns for u, v in zip(coverage, coverage[1:])):
                raise ValueError("base tracking gap inside cycle")
            dx, dy = b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]
            c.update(valid_displacement=True, dx_m=dx, dy_m=dy,
                     forward_m=dx*math.cos(a["yaw_rad"]) + dy*math.sin(a["yaw_rad"]),
                     lateral_m=-dx*math.sin(a["yaw_rad"]) + dy*math.cos(a["yaw_rad"]),
                     net_m=math.hypot(dx, dy), yaw_change_rad=wrap(b["yaw_rad"]-a["yaw_rad"]))
            c["forward_m_s"] = c["forward_m"] / c["duration_s"]
            c["unassisted_completed"] = not (c["fall_events"] or c["aborted"] or c["external_assistance"])
        except ValueError as error:
            c["reason"] = str(error)
        cycles.append(c)
    valid = [c for c in cycles if c["valid_displacement"]]
    eligible = [c for c in valid if c["unassisted_completed"]]
    values = [c["forward_m"] for c in eligible]
    reviewed = meta.get("review", {}).get("falls_reviewed") is True
    assistance_reviewed = meta.get("review", {}).get("external_assistance_reviewed") is True
    uncertainty = meta["measurement"].get("xy_endpoint_uncertainty_m")
    if uncertainty is not None and (type(uncertainty) not in (int, float) or uncertainty < 0):
        raise ValueError("Endpoint uncertainty must be a nonnegative bound")
    report = {
        "session_id": meta["session_id"], "evidence": meta["evidence"],
        "control_mode": meta["control_mode"], "cycles": cycles,
        "total_cycles": len(cycles), "measurable_cycles": len(valid),
        "completed_cycles_without_flagged_incidents": len(eligible),
        "mean_forward_m_per_cycle": statistics.mean(values) if values else None,
        "sample_sd_forward_m": statistics.stdev(values) if len(values) > 1 else None,
        "fall_events_in_session": sum(e["data"]["name"] == "fall" for e in events),
        "fall_fraction_complete_cycles": sum(c.get("fall_events", 0) > 0 for c in cycles)/len(complete) if reviewed and complete else None,
        "review_complete": reviewed and assistance_reviewed,
        "displacement_uncertainty_bound_m": 2*uncertainty if uncertainty is not None else None,
        "notice": "Synthetic fixture, not robot evidence." if meta["evidence"] == "synthetic" else "Metrics describe the labelled source; they do not establish autonomy.",
    }
    return report


def replay_plan(rows, max_command_gap_s=0.25):
    """Return a schedule only. Never connects to a robot or invents target mapping."""
    validate(rows)
    commands = [r for r in rows[1:] if r["kind"] == "command" and r["data"]["stage"] == "submitted"]
    if not commands:
        raise ValueError("No submitted SDK targets. Quest packets are not robot commands.")
    origin = commands[0]["t_ns"]
    if any(not 0 < b["t_ns"]-a["t_ns"] <= max_command_gap_s*1e9 for a, b in zip(commands, commands[1:])):
        raise ValueError("Duplicate timestamp or gap in command stream; inspect before replay")
    if any(r["kind"] == "event" and r["data"]["name"] in {"fall", "abort", "external_assistance"} for r in rows[1:]):
        raise ValueError("Incident in recording; choose a reviewed clean segment")
    return {"mode": "offline_schedule_only", "evidence": rows[0]["evidence"],
            "source_session_id": rows[0]["session_id"],
            "commands": [{"offset_ns": r["t_ns"]-origin, "target": r["data"]["target"]} for r in commands],
            "hardware_ready": False}
