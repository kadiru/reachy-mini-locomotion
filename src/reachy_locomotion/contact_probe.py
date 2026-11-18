"""CPU-only free-foot contact experiment. No SDK, network, or hardware control.

This is a physics plumbing probe, not a gait or validated digital twin. A
zero-command hold and an explicitly external push test support and mobility.
"""
import argparse
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

PIN = "a9a2a42adcc1abc7d9c90d390889bbebf930c96c"
MODEL_PATH = "src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml"
BASE = "body_foot_3dprint"


def validate_config(config):
    if config.get("schema_version") != 1 or config.get("evidence") != "uncalibrated_simulation":
        raise ValueError("Expected version 1 uncalibrated_simulation config")
    if config.get("upstream_commit") != PIN:
        raise ValueError("Upstream revision differs from the inspected model")
    for key in ("foot_radius_m", "foot_half_height_m", "initial_clearance_m",
                "position_kp", "position_kv", "motor_force_limit_nm", "timestep_s"):
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("Expected positive finite " + key)
    friction = config.get("friction_slide_torsion_roll")
    if not isinstance(friction, list) or len(friction) != 3 or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0 for x in friction
    ):
        raise ValueError("Expected three finite nonnegative friction coefficients")
    if not 0.0001 <= config["timestep_s"] <= 0.002:
        raise ValueError("Probe timestep must be between 0.1 and 2 ms")
    return config


def scene_xml(upstream, config, free=True):
    """Derive in memory; never modify the official checkout or bundle its assets."""
    validate_config(config)
    upstream = Path(upstream).resolve()
    commit = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
    if commit != PIN:
        raise ValueError("Checkout must be pinned to " + PIN)
    original = (upstream / MODEL_PATH).read_bytes()
    pinned = subprocess.check_output(["git", "-C", str(upstream), "show", PIN + ":" + MODEL_PATH])
    if original != pinned:
        raise ValueError("Official MJCF has local changes; preserve it and use a clean pinned checkout")
    root = ET.fromstring(original)
    root.set("model", "reachy_uncalibrated_contact_probe")
    root.find("compiler").set("meshdir", str((upstream / MODEL_PATH).parent / "assets"))
    ET.SubElement(root, "option", timestep=str(config["timestep_s"]), integrator="implicitfast",
                  solver="Newton", iterations="100", tolerance="1e-10", gravity="0 0 -9.81")
    world = root.find("worldbody")
    base = world.find("body[@name='%s']" % BASE)
    if base is None or base.find("joint") is not None or base.find("freejoint") is not None:
        raise ValueError("Expected the inspected fixed-foot model")
    if free:
        base.set("pos", "0 0 " + str(config["initial_clearance_m"]))
        ET.SubElement(base, "freejoint", name="floating_foot")
    # Explicit inertia already exists on the foot. mass=0 keeps proxy geometry
    # from contributing an invented second mass if compilation settings change.
    friction = " ".join(map(str, config["friction_slide_torsion_roll"]))
    ET.SubElement(base, "geom", name="foot_contact_proxy", type="cylinder",
                  size="%s %s" % (config["foot_radius_m"], config["foot_half_height_m"]),
                  pos="0 0 " + str(config["foot_half_height_m"]), mass="0",
                  contype="1", conaffinity="1", condim="6", friction=friction,
                  rgba="0.2 0.7 0.8 0.5", group="3")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 0.05",
                  contype="1", conaffinity="1", condim="6", friction=friction,
                  rgba="0.2 0.25 0.3 1")
    for actuator in root.find("actuator"):
        actuator.set("kp", str(config["position_kp"]))
        actuator.set("kv", str(config["position_kv"]))
        limit = config["motor_force_limit_nm"]
        actuator.set("forcerange", "%s %s" % (-limit, limit))
        actuator.set("forcelimited", "true")
    if "motor_model" in config:
        from .motors import modify_physics_xml
        modify_physics_xml(root, config["motor_model"])
    return ET.tostring(root, encoding="unicode"), hashlib.sha256(original).hexdigest()


def relative_head_pose(model, data):
    """T_foot_head in raw MJCF coordinates, NOT an SDK target/reference frame."""
    import numpy as np
    base = model.body(BASE).id
    head = model.site("head").id
    rotation = data.xmat[base].reshape(3, 3)
    result = np.eye(4)
    result[:3, :3] = rotation.T @ data.site_xmat[head].reshape(3, 3)
    result[:3, 3] = rotation.T @ (data.site_xpos[head] - data.xpos[base])
    return result


def initialize(model, config, fixed_xml):
    """Close the Stewart chain while anchored, then release without momentum.

    This artificial reset is excluded from all reported travel. It is not a
    model of a robot waking up or a demonstrated recovery behavior.
    """
    import mujoco
    import numpy as np
    fixed = mujoco.MjModel.from_xml_string(fixed_xml)
    anchored = mujoco.MjData(fixed)
    types, affinities = fixed.geom_contype.copy(), fixed.geom_conaffinity.copy()
    fixed.geom_contype[:] = 0
    fixed.geom_conaffinity[:] = 0
    # Use CAD neutral joint targets, not guessed task-space inverse kinematics.
    anchored.ctrl[:] = 0
    for _ in range(round(1.0 / config["timestep_s"])):
        mujoco.mj_step(fixed, anchored)
    fixed.geom_contype[:] = types
    fixed.geom_conaffinity[:] = affinities
    for _ in range(round(0.5 / config["timestep_s"])):
        mujoco.mj_step(fixed, anchored)
    if not np.isfinite(anchored.qpos).all() or any(w.number for w in anchored.warning):
        raise ValueError("Anchored initialization produced a numerical warning")
    data = mujoco.MjData(model)
    for j in range(fixed.njnt):
        target = model.joint(fixed.joint(j).name).id
        count = 4 if fixed.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL else 1
        old, new = fixed.jnt_qposadr[j], model.jnt_qposadr[target]
        data.qpos[new:new + count] = anchored.qpos[old:old + count]
    data.qvel[:] = 0
    data.ctrl[:] = anchored.ctrl
    mujoco.mj_forward(model, data)
    # Let the foot settle on the plane before measuring either trial.
    for _ in range(round(1.0 / config["timestep_s"])):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
        raise ValueError("Free-foot settling produced a numerical warning")
    return data


def run_probe(upstream, config, seconds=2.0, push_n=2.0):
    import mujoco
    import numpy as np
    if not math.isfinite(seconds) or not 0.5 <= seconds <= 10:
        raise ValueError("Probe duration must be 0.5–10 seconds")
    if not math.isfinite(push_n) or not 0 <= push_n <= 10:
        raise ValueError("External test force must be 0–10 N")
    xml, source_hash = scene_xml(upstream, config)
    fixed_xml, _ = scene_xml(upstream, config, free=False)
    model = mujoco.MjModel.from_xml_string(xml)
    base = model.body(BASE).id
    head_body = int(model.site_bodyid[model.site("head").id])
    foot_geom = model.geom("foot_contact_proxy").id
    floor = model.geom("floor").id
    steps = round(seconds / model.opt.timestep)
    trials = []
    for label, force in (("zero_target_hold", 0.0), ("external_push_not_locomotion", push_n)):
        data = initialize(model, config, fixed_xml)
        start = data.xpos[base].copy()
        clock_start = data.time
        max_tilt = max_closure = 0.0
        support_steps = 0
        trajectory = []
        for step in range(steps):
            elapsed = step * model.opt.timestep
            data.xfrc_applied[:] = 0
            # 100 ms artificial lateral force at the head body's COM. This is
            # expressly NOT an actuator or a candidate walking controller.
            if 0.2 <= elapsed < 0.3:
                data.xfrc_applied[head_body, 1] = force
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)  # poses and contacts at current qpos
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise ValueError("Nonfinite rollout state")
            tilt = math.acos(float(np.clip(data.xmat[base].reshape(3, 3)[2, 2], -1, 1)))
            max_tilt = max(max_tilt, tilt)
            closure = max(np.linalg.norm(data.site_xpos[model.site("closing_%d_1" % i).id]
                                         - data.site_xpos[model.site("closing_%d_2" % i).id]) for i in range(1, 6))
            max_closure = max(max_closure, float(closure))
            supported = any({int(c.geom1), int(c.geom2)} == {foot_geom, floor} and c.efc_address >= 0 for c in data.contact)
            support_steps += int(supported)
            if step % max(1, round(0.02 / model.opt.timestep)) == 0 or step == steps - 1:
                trajectory.append({"t_s": float(data.time - clock_start), "foot_xyz_m": data.xpos[base].tolist(),
                                   "foot_tilt_rad": tilt, "foot_floor_contact": supported})
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(data.warning) if w.number}
        if warnings:
            raise ValueError("MuJoCo numerical warnings: " + str(warnings))
        trials.append({"name": label, "external_force_y_n": force,
                       "measurement_duration_s": float(data.time - clock_start),
                       "foot_displacement_world_m": (data.xpos[base] - start).tolist(),
                       "max_foot_tilt_rad": max_tilt, "max_loop_closure_error_m": max_closure,
                       "foot_floor_contact_fraction": support_steps / steps,
                       "final_T_foot_head_mjcf": relative_head_pose(model, data).tolist(),
                       "warnings": warnings, "trajectory": trajectory})
    return {"evidence": "uncalibrated_simulation", "hardware_ready": False, "gait_demonstrated": False,
            "upstream_commit": PIN, "upstream_xml_sha256": source_hash, "mujoco_version": mujoco.__version__,
            "config": config, "nq": model.nq, "nv": model.nv, "nu": model.nu, "neq": model.neq,
            "free_joint_count": int(sum(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)),
            "base_subtree_mass_kg": float(model.body_subtreemass[base]),
            "training_jobs_started": 0, "trials": trials}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    parser.add_argument("--config", default="configs/contact-proxy.json")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--push-n", type=float, default=2.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        output = Path(args.output)
        if output.exists():
            raise FileExistsError("Choose a new report path; existing reports are preserved")
        config = json.loads(Path(args.config).read_text())
        result = run_probe(args.upstream, config, args.seconds, args.push_n)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print("Saved uncalibrated contact probe: " + str(output))
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(2, "Error: " + str(error) + "\n")


if __name__ == "__main__":
    main()
