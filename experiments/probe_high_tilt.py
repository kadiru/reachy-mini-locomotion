"""Test a bounded head movement to return flat from a prescribed side tilt.

These are simulation resets on the loaded rim. The first feedback sample is
delayed one 50 Hz tick. The catch moves head roll and lateral position, then restores neutral height.
"""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.geometry import rotation
from reachy_locomotion.primitives import Primitive, PrimitiveSimulator, blend

SIMULATORS = {}


class TiltCatch:
    def __init__(self, case):
        self.case = case
        self.previous = 0.0
        self.armed = False
        self.neutral_start = None
        self.neutral_origin = np.zeros(2)
        self.catch_trigger = self.yaw_trigger = None

    def target(self, t, roll):
        sensed, self.previous = self.previous, roll
        c = self.case
        side = c["side"]
        if side*sensed > 34:
            self.armed = True
        initial = np.array(c.get("initial_head", [0, 0, 0])[:2])
        first = np.array(c["first"])
        second = np.array(c["second"])
        if not c.get('hold_setup') and self.catch_trigger is None and t >= .02 and side*sensed >= c.get("trigger_at", -90):
            self.catch_trigger = t
        elapsed = 0 if self.catch_trigger is None else t-self.catch_trigger
        if elapsed < c["switch"]:
            command = initial + (first-initial)*blend(elapsed / c["rise"])
        else:
            u = blend((elapsed-c["switch"]) / c["return_time"])
            at_switch = initial + (first-initial)*blend(c["switch"]/c["rise"])
            command = (1-u)*at_switch + u*second
        if self.armed and self.neutral_start is None and side*sensed < c["neutral_at"]:
            self.neutral_start = t
            self.neutral_origin = command.copy()
        if self.neutral_start is not None:
            command = self.neutral_origin * (1-blend((t-self.neutral_start)/.4))
        if c.get("hold_setup"):
            command = initial
            self.neutral_start = None
        pose = np.eye(4)
        pose[:3,:3] = rotation(np.radians([side*command[0],0,0]))
        z = c.get("initial_head",[0,0,0])[2]/1000
        z *= 1-blend((t-self.neutral_start)/.4) if self.neutral_start is not None else 1
        pose[:3,3] = [0, side*command[1]/1000, .177+z]
        stage = "held head pose" if c.get("hold_setup") else "tilt catch"
        if self.neutral_start is not None:
            stage = "neutral return"
        return pose, stage, sensed


def run(case):
    dt = case["dt"]
    if dt not in SIMULATORS:
        config = json.loads(Path("configs/motors-speed-trained.json").read_text())
        config["timestep_s"] = dt
        sim = PrimitiveSimulator("experiments/upstream-reachy-mini", config)
        SIMULATORS[dt] = (sim, sim.initial_targets.copy())
    sim, neutral_targets = SIMULATORS[dt]
    # The simulator physically warms up the servo on the floor before the
    # artificial base reset. Use a fresh target vector for every case.
    sim.initial_targets = neutral_targets.copy()
    if "initial_head" in case:
        roll,y,z = case["initial_head"]
        pose = np.eye(4)
        pose[:3,:3] = rotation(np.radians([case["side"]*roll,0,0]))
        pose[:3,3] = [0,case["side"]*y/1000,.177+z/1000]
        try:
            sim.initial_targets[sim.act] = sim.checked_ik(pose)
        except ValueError:
            return dict(case=case, settled=False, completed=False, termination="infeasible setup pose")
    r = sim.run_primitive(Primitive(), seconds=case.get("seconds",4),
        waist_mode="hold_neutral", controller=TiltCatch(case), tilt_limit_deg=case.get("tilt_limit_deg"),
        diagnostic_initial_roll_deg=case["side"]*case["angle"],
        diagnostic_initial_angular_rate_deg_s=case["side"]*case["rate"],
        record=bool(case.get("record")))
    if case.get("record"):
        with Path(case["record"]).open("x") as f:
            json.dump(r,f,allow_nan=False)
    return dict(case=case, trigger_s=None if case.get("hold_setup") else r["catch_trigger_s"], **{k:r[k] for k in ("completed","settled","termination","elapsed_seconds",
                "peak_tilt_deg","min_joint_margin_rad","max_closure_error_m","waist_peak_torque_nm")},
                final_roll_deg=float(np.degrees(r["terminal_state"]["foot_rpy_rad"][0])))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--jobs",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--workers",type=int,default=4)
    a = p.parse_args()
    jobs = json.loads(a.jobs.read_text())
    with a.output.open("x") as out, ProcessPoolExecutor(a.workers) as pool:
        for r in pool.map(run,jobs):
            print(json.dumps(r,allow_nan=False),file=out,flush=True)


if __name__ == "__main__":
    main()
