"""Reproduce bounded CPU sway search, sensitivity checks, and controls."""
import argparse
import copy
import itertools
import json
from dataclasses import asdict
from pathlib import Path

from reachy_locomotion.sway import Gait, SwaySimulator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["search", "robustness", "controls", "all"], default="all")
    parser.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Choose a new report path")
    base = json.loads(Path("configs/sway-cad-proxy.json").read_text())
    candidate = Gait(**json.loads(Path("configs/sway-candidate.json").read_text()))
    jobs = []
    if args.stage in ("search", "all"):
        for radius, rolls, frequencies, cycles in [(.075, [20,35], [1.5,3,4.5], 6), (.056, [25,35,40], [2,3,4.5], 8)]:
            for torque, roll, frequency, pattern in itertools.product([.6,1.2,2.4], rolls, frequencies, [(0,0),(30,90),(30,-90)]):
                config = dict(base, foot_radius_m=radius, motor_force_limit_nm=torque)
                config["notes"] = "Uncalibrated search hypothesis; each numeric parameter is recorded explicitly."
                jobs.append(("search", config, Gait(roll_deg=roll, frequency_hz=frequency, yaw_deg=pattern[0], yaw_phase_deg=pattern[1]), cycles))
    if args.stage in ("robustness", "all"):
        for torque, roll, frequency in [(1.2,25,4.5),(1.2,40,4.5),(2.4,40,3)]:
            for dt, friction in itertools.product([.001,.002], [.4,.7,1.0]):
                config = dict(base, motor_force_limit_nm=torque, timestep_s=dt, friction_slide_torsion_roll=[friction,.005,.0001])
                config["notes"] = "Uncalibrated sensitivity test; each numeric parameter is recorded explicitly."
                jobs.append(("robustness", config, Gait(roll_deg=roll,frequency_hz=frequency,yaw_deg=30,yaw_phase_deg=90),12))
    if args.stage in ("controls", "all"):
        for dt in [.001,.002]:
            config = dict(base,timestep_s=dt)
            for name, updates in [("zero_motion",dict(roll_deg=0,yaw_deg=0)),("roll_only",dict(yaw_deg=0)),
                                  ("candidate",{}),("mirror",dict(mirror=-1)),("reverse_yaw_phase",dict(yaw_phase_deg=-90))]:
                jobs.append((name, config, Gait(**dict(asdict(candidate),**updates)),12))
        for torque in [.6,.9,1.5]:
            jobs.append(("torque_sensitivity",dict(base,motor_force_limit_nm=torque),candidate,12))
        for radius in [.051,.063,.075]:
            jobs.append(("radius_sensitivity",dict(base,foot_radius_m=radius),candidate,12))
        jobs.append(("longer_candidate",base,candidate,30))
    results = []
    simulator = None
    previous = None
    for index, (label, config, gait, cycles) in enumerate(jobs):
        if config != previous:
            simulator = SwaySimulator(args.upstream, config)
            previous = copy.deepcopy(config)
        result = simulator.run(gait,cycles=cycles)
        result["study_label"] = label
        results.append(result)
        if label != "search" or index % 20 == 0:
            print(index+1,len(jobs),label,result["valid"],"forward_mm",round(result["forward_m"]*1000,2),
                  "lateral_mm",round(result["lateral_m"]*1000,2),result["failure_reason"],flush=True)
    report = {"evidence":"uncalibrated_simulation", "method":"scripted_parameter_sweep_not_RL",
              "stage":args.stage, "trial_count":len(results), "results":results}
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("x") as file:
        json.dump(report,file,indent=2,allow_nan=False)
    print("Saved",output)


if __name__ == "__main__":
    main()
