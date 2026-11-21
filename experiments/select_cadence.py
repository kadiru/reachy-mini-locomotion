"""Select a low-cadence walker using only the bilateral nominal rollouts."""
import argparse
import math
from pathlib import Path

from train_swing_distance import read, save


def select(root):
    finalists = []
    for name in ("first", "impulse", "redirect", "lift", "forward"):
        folder = root / name
        policy = read(folder / "policy.json")
        stage = "confirmation" if name == "first" else "nominal"
        rows = [r for r in read(folder / "history.json")[stage]
                if r["name"] == policy["selected_candidate"]]
        if sorted(r["side"] for r in rows) != [-1, 1]:
            raise ValueError(f"{name} requires both nominal starting sides")
        passed = sum(r["passed"] for r in rows)
        edge = sum(r["passed"] and len(r["edge_brake_strokes"]) >= 2 for r in rows)
        worst = min(r["mm_per_active_s"] / math.sqrt(r["cadence_hz"]) for r in rows)
        finalists.append(((passed, edge, worst), name, policy, rows))
    (passed, _, _), name, policy, rows = max(finalists, key=lambda f: f[0])
    if passed != 2:
        raise ValueError("No candidate passed from both starting sides")
    video = max(rows, key=lambda r: (len(r["edge_brake_strokes"]) >= 2, r["distance_mm"]))
    summary = dict(selected=name, side=video["side"],
                   table=[dict(experiment=n, passed=k[0], edge_brake_pass=k[1],
                               worst_weighted_speed=k[2]) for k, n, _, _ in finalists])
    selected = dict(policy, source_experiment=name, waist_mode="hold_neutral", command_hz=50,
                    max_counted_active_rate_hz=1.3, minimum_same_axis_event_spacing_s_exclusive=.50,
                    objective="Maximize signed forward mm/s divided by the square root of directional swings/s. Prefer a passing gait on both sides, then repeated braking on a loaded edge.")
    return selected, summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    policy, summary = select(args.root)
    save(args.output, policy)
    save(args.root / "selection.json", summary)
    print(summary)
