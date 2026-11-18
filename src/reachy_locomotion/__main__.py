import argparse
import json
import math
import statistics
from pathlib import Path
from .analysis import analyze, replay_plan
from .capture import poll_state
from .recording import Recorder, load, validate


def synthetic_fixture():
    """Construct a known-answer fixture. Its 10 mm/cycle is invented."""
    meta = {"kind": "session", "schema_version": 1, "session_id": "SYNTHETIC-001",
            "evidence": "synthetic", "control_mode": "teleoperation",
            "robot": {"variant": "synthetic_wireless", "sdk_commit": None},
            "setup": {"surface": "invented", "condition_id": "synthetic-only"},
            "clock": {"type": "synthetic", "origin_monotonic_ns": 0},
            "measurement": {"max_pose_gap_s": 0.11, "xy_endpoint_uncertainty_m": 0.0005},
            "review": {"falls_reviewed": True, "external_assistance_reviewed": True}}
    rows = [meta]
    def add(kind, t, data):
        rows.append({"kind": kind, "t_ns": round(t*1e9), "source": "SYNTHETIC_GENERATOR", "data": data})
    for cycle in range(3):
        add("event", cycle, {"name": "cycle_start", "cycle_id": str(cycle)})
        add("event", cycle+1, {"name": "cycle_end", "cycle_id": str(cycle)})
    for i in range(31):
        t = i/10
        add("base_pose", t, {"tracked_body": "foot", "frame": "world", "x_m": .01*t, "y_m": .002*math.sin(2*math.pi*t), "yaw_rad": 0})
        add("command", t, {"stage": "submitted", "target": {"body_yaw": .05*math.sin(2*math.pi*t)}})
        add("state", t, {"sdk_state": {"body_yaw": .04*math.sin(2*math.pi*t)}})
        add("video_frame", t, {"stream_id": "synthetic-no-video-file", "frame_index": i, "pts_s": t, "clock_mapping": "synthetic_identity"})
    rows = [meta] + sorted(rows[1:], key=lambda r: r["t_ns"])
    return validate(rows)


def main():
    import sys
    from .first_session import COMMANDS
    if len(sys.argv) > 1 and sys.argv[1] in COMMANDS:
        from .first_session import main as first_session_main
        return first_session_main(sys.argv[1:])
    parser = argparse.ArgumentParser(description="Reachy locomotion evidence tools and explicit first-session controls",
        epilog="First-session commands (use COMMAND --help): " + ", ".join(sorted(COMMANDS)))
    sub = parser.add_subparsers(dest="cmd", required=True)
    fixture = sub.add_parser("synthetic")
    fixture.add_argument("output")
    for name in ("validate", "analyze", "replay-plan"):
        p = sub.add_parser(name)
        p.add_argument("input")
        p.add_argument("--output")
    compare = sub.add_parser("compare")
    compare.add_argument("inputs", nargs="+")
    compare.add_argument("--output")
    state = sub.add_parser("capture-state")
    state.add_argument("--url", required=True)
    state.add_argument("--metadata", required=True)
    state.add_argument("--output", required=True)
    state.add_argument("--seconds", type=float, default=10)
    state.add_argument("--hz", type=float, default=20)
    audit = sub.add_parser("audit-sim")
    audit.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    audit.add_argument("--output")
    args = parser.parse_args()
    try:
        if args.cmd == "synthetic":
            Path(args.output).parent.mkdir(parents=True, exist_ok=True)
            with open(args.output, "x") as f:
                for row in synthetic_fixture():
                    f.write(json.dumps(row, allow_nan=False) + "\n")
            print("Created SYNTHETIC fixture; no robot was involved.")
            return
        if args.cmd == "capture-state":
            meta = json.loads(Path(args.metadata).read_text())
            with Recorder(args.output, meta) as rec:
                result = poll_state(rec, args.url, args.seconds, args.hz)
            print(json.dumps(result))
            if result["state_samples"] == 0:
                raise ValueError("No states captured; error log saved. No sensor evidence collected.")
            print("Read-only capture saved; timestamps are host receipt times.")
            return
        if args.cmd == "audit-sim":
            from .sim_audit import audit_model
            result = audit_model(args.upstream)
        elif args.cmd == "compare":
            recordings = [load(p) for p in args.inputs]
            identities = [r[0]["session_id"] for r in recordings]
            if len(set(identities)) != len(identities):
                raise ValueError("Duplicate sessions cannot be counted as independent trials")
            results = [analyze(r) for r in recordings]
            conditions = {json.dumps({k: r[0][k] for k in ("evidence", "control_mode", "robot", "setup", "measurement")}, sort_keys=True) for r in recordings}
            if len(conditions) != 1:
                raise ValueError("Compare requires matching evidence, control, robot, setup and measurement conditions")
            means = [r["mean_forward_m_per_cycle"] for r in results if r["mean_forward_m_per_cycle"] is not None]
            result = {"sessions": results, "independent_trial_count": len(means),
                      "between_trial_sd_m_per_cycle": statistics.stdev(means) if len(means)>1 else None,
                      "mean_of_trial_means_m_per_cycle": statistics.mean(means) if means else None}
        else:
            rows = load(args.input)
            result = {"valid": True, "records": len(rows), "evidence": rows[0]["evidence"]} if args.cmd == "validate" else analyze(rows) if args.cmd == "analyze" else replay_plan(rows)
        rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if args.output:
            Path(args.output).write_text(rendered)
            print("Saved " + args.output)
        else:
            print(rendered, end="")
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, "Error: " + str(error) + "\n")


if __name__ == "__main__":
    main()
