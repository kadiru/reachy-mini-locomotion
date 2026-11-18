# Official MuJoCo audit

October 28, 2026 (America/Los_Angeles). Official source pinned at `a9a2a42adcc1abc7d9c90d390889bbebf930c96c`. Compiled locally with MuJoCo 3.3.7 on CPU; no dynamics rollout, GUI, GPU job or training. Machine-readable results: `experiments/reports/stock-mujoco-audit.json`.

The official [simulation guide](https://huggingface.co/docs/reachy_mini/en/platforms/simulation/get_started) documents the `reachy-mini[mujoco]` extra and `reachy-mini-daemon --sim`. macOS GUI uses `mjpython -m reachy_mini.daemon.app.main --sim`. We compiled the MJCF directly, avoiding a daemon/media installation. API prototyping support does not establish locomotion fidelity.

## Inspected effective model

| Check | Verified result | Consequence |
|---|---|---|
| Root mobility | `body_foot_3dprint` has zero joints; no free joints in empty scene | Foot is fixed to world. It cannot reproduce observed physical base travel. |
| Foot contact | Zero collision-enabled geoms directly on root foot | Adding a free joint alone would not provide the observed foot/surface contact. |
| Other contact | Decomposed collision meshes exist on upper body/head; floor plane exists | These support some collision checking, not a calibrated rocking foot. |
| Friction | Compiled floor `[1.0, 0.005, 0.0001]` (slide/torsion/roll) | Defaults are not measurements of the user's surface. `joint frictionloss` is not floor friction. |
| Selected actuators | Nine position servos inherit `chosen_actuator` from `perfect_actuator`, each force range `[-20,20]` | Do not interpret this as the real motor torque envelope. Other named actuator classes in the file are not selected. |
| Joint/control limits | Body + six Stewart actuators inherit joint ranges; both antenna controls are unlimited in compiled model | Keep real SDK/firmware constraints; this is not a hardware command envelope. |
| Inertia | Compiled root subtree mass 1.17652824 kg | Compare actual Wireless mass, battery, pads and mass distribution; this value is model data, not a hardware weighing. |
| Dimensions | `nq=37`, `nv=30`, `nu=9`; passive joints and closure constraints exist | Do not assume a simple nine-independent-joint rigid chain or remove Stewart closure constraints. |
| Daemon stepping | Backend sets 2 ms steps and decimation 10 (50 Hz target loop) | Reproduce transport/actuation timing before optimizing a fast swing. |
| Initialization | Backend temporarily disables unnamed collisions, then reenables | A free-base initialization needs a separately tested support/settling procedure. |
| Head pose reporting | World site transform with constant z offset (`0.177`) | Moving the root requires an explicit robot-reference transform; otherwise FK/IK mixes moving world and base frames. |

Primary source files: [MJCF](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml), [empty scene](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/descriptions/reachy_mini/mjcf/scenes/empty.xml), [backend](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/backend/mujoco/backend.py). MuJoCo's [free-joint/reference documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-freejoint) and [contact computation reference](https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection) define the underlying model behavior.

## Before locomotion simulation

November 18 update: a separate uncalibrated free-foot contact experiment now
exists; see [CONTACT_PROBE.md](CONTACT_PROBE.md). It implements the free joint,
placeholder collider, raw MJCF foot/head frame transform and short hold/push
checks. Measured geometry, actuator identification, SDK-frame replay and policy
learning remain open. The list below records the original October audit scope.

Create a separate experimental scene, preserving this pinned stock model as the baseline. Required work, **not yet implemented**:

1. Add a free joint to the physical foot/root and preserve all relative links and closed-chain constraints. Keep SDK task poses in a documented base reference: convert world site pose using `inverse(T_world_base) @ T_world_head`, then apply the SDK reference-origin convention.
2. Model the actual underside, rim/pads and likely alternating contact patches with explicit simple convex collision geometry. Validate against photos/measurements and rocking video. Avoid relying on a single convex hull if it fills a concavity or changes the support footprint.
3. Measure mass, approximate center of mass and neutral support polygon. Calibrate tilt response and quasi-static rocking first, then match observed swing response.
4. Fit friction/contact compliance, restitution, damping and rolling/torsional resistance. Vary them over plausible measured ranges. Check direction/speed stability across timestep and solver settings; reject solver-dependent travel or energy injection.
5. Identify installed motors and realistic position tracking, velocity, acceleration, torque/current limits, saturation, deadband and control latency. Do not use the perfect actuator to claim sim-to-real success.
6. Replay measured targets while comparing joint trajectories, IMU orientation/rate, contact phase and external displacement. Keep held-out captures for validation.
7. Only then expose a Gym-style environment and evaluate an optimizer or policy. A model that moves under a free joint is an experiment, not evidence that the physical mechanism has been reproduced.

## Reproduction

Already present locally: `.venv`, pinned clone and hydrated meshes. Original clone downloaded LFS pointer files; those failed MuJoCo compilation. The local helper fetched and SHA256/size-verified 65 official assets (27,448,841 bytes), including nested collision meshes. This resolved both initial missing-mesh failures.

For a fresh setup (network required):

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r experiments/requirements-audit.lock
git clone https://github.com/pollen-robotics/reachy_mini.git experiments/upstream-reachy-mini
git -C experiments/upstream-reachy-mini checkout a9a2a42adcc1abc7d9c90d390889bbebf930c96c
.venv/bin/python experiments/fetch_mujoco_assets.py
PYTHONPATH=src .venv/bin/python -m reachy_locomotion audit-sim --output experiments/reports/stock-mujoco-audit.json
```

The helper only hydrates Git LFS pointers with matching hashes. It does not alter robot geometry or the source model. `requirements-audit.lock` records this Mac/Python 3.9 environment; other platforms may need different compatible package versions.
