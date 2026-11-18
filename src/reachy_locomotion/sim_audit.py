"""Compile the unchanged upstream MJCF on CPU and inspect effective parameters."""
import subprocess
from pathlib import Path


def audit_model(upstream):
    import mujoco
    root = Path(upstream).resolve()
    scene = root / "src/reachy_mini/descriptions/reachy_mini/mjcf/scenes/empty.xml"
    model = mujoco.MjModel.from_xml_path(str(scene))
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body_foot_3dprint")
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if base_id < 0 or floor_id < 0:
        raise ValueError("Upstream body names changed; re-inspect model")
    contact_geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] == base_id and (model.geom_contype[i] or model.geom_conaffinity[i])]
    free = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    return {
        "evidence": "compiled_model_inspection_only",
        "upstream_commit": subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip(),
        "mujoco_version": mujoco.__version__, "scene": str(scene),
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "free_joints": free,
        "base_joint_count": int(model.body_jntnum[base_id]),
        "base_collision_geom_count": len(contact_geoms),
        "base_subtree_mass_kg": float(model.body_subtreemass[base_id]),
        "floor_friction_slide_torsion_roll": model.geom_friction[floor_id].tolist(),
        "actuators": [{"name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i),
                       "force_limited": bool(model.actuator_forcelimited[i]),
                       "force_range": model.actuator_forcerange[i].tolist(),
                       "control_limited": bool(model.actuator_ctrllimited[i]),
                       "control_range": model.actuator_ctrlrange[i].tolist()} for i in range(model.nu)],
        "locomotion_ready": False,
        "reason": "Fixed foot; no free joint or foot contact collider. Actuators and floor need hardware calibration.",
        "simulation_rollouts_run": 0, "training_jobs_started": 0,
    }
