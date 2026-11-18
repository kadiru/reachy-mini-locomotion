# Moving-foot MuJoCo experiment

This experiment adds a free root and explicit foot/floor contact to the pinned
official Reachy Mini model. It is **uncalibrated simulation**, not a trained gait
or a model ready for sim-to-real transfer.

The stock model has a fixed foot with no foot collision shape. The derived
scene preserves all nine actuators, the five Stewart closure constraints and
the original 1.17652824 kg model mass. `scene_xml` creates the scene in memory;
it checks the upstream commit and original XML and never modifies the checkout.
Meshes remain external assets under their original licensing.

## Run

From the repository root, with Python 3.9–3.12 (tested locally on 3.9):

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[capture,sim]'
git init experiments/upstream-reachy-mini
git -C experiments/upstream-reachy-mini remote add origin https://github.com/pollen-robotics/reachy_mini.git
git -C experiments/upstream-reachy-mini fetch --depth 1 origin a9a2a42adcc1abc7d9c90d390889bbebf930c96c
GIT_LFS_SKIP_SMUDGE=1 git -C experiments/upstream-reachy-mini checkout --detach FETCH_HEAD
.venv/bin/python experiments/fetch_mujoco_assets.py
.venv/bin/python -m reachy_locomotion.contact_probe --output experiments/reports/contact-probe-local.json
```

The checkout commands are for a new checkout; reuse an existing clean checkout
at that commit. LFS hydration verifies file hashes and sizes. The probe runs on
CPU without a viewer, network, daemon, robot connection or training process.
Reports refuse overwriting. Use a new output name for each run.

## What is and is not modeled

`configs/contact-proxy.json` explicitly marks every added value as unmeasured:
75 mm foot radius, 4 mm half-height, friction, 0.6 Nm actuator cap and servo gains.
The simple cylinder is a disposable contact proxy, not a reconstruction of the
actual underside, pads or rim. The model has no cable and no measured Wireless
battery/center-of-mass correction. Joint damping/friction/inertias are retained
from upstream; motor speed limits, saturation dynamics and latency remain work.

The chain first settles with its foot anchored, then with a free foot under
gravity. This artificial reset is excluded from reported displacement. After
settling, both two-second trials hold zero CAD joint targets:

1. A no-push baseline checks spontaneous drift and support.
2. A 2 N lateral push at the head body's center of mass lasts 100 ms. This is
   explicitly an external perturbation, **not head-actuated locomotion**.

The report stores world foot displacement, foot tilt, foot/floor active contact
fraction, closure error, solver warnings and a 50 Hz trajectory. It includes
`hardware_ready: false` and `gait_demonstrated: false`. Numerical warnings fail
the run rather than quietly accepting MuJoCo's automatic state reset.

`relative_head_pose` returns raw MJCF `T_foot_head`; this is invariant to world
translation/yaw. It is not an SDK pose. The SDK origin/sign mapping and reachable
head trajectories must be validated before replaying recorded SDK targets.

## First results — November 18, 2026

`experiments/reports/contact-probe-2026-11-18.json`: 44 position coordinates,
36 velocity coordinates, one free joint, nine actuators and five equalities.
Both trials completed without numerical warnings and with foot contact at all
sampled simulation steps. No-push horizontal drift was below 1 micrometre;
maximum closure error across both trials was approximately 1.85 micrometres.
The externally pushed trial moved about 0.198 mm laterally and barely tilted.
These are computed behavior of the proxy only; **no penguin gait is reproduced**.

A second run at 1 ms (`contact-probe-1ms-2026-11-18.json`) also has no warnings,
with approximately 0.198 mm pushed travel and 0.47 micrometre maximum closure
error. This is a limited timestep check of hold/push behavior, not validation
of an oscillating gait or contact identification.

Before optimizing travel, replace the proxy with measured support geometry,
validate forward direction/SDK frames, fit motor and contact response to recorded
motion, and check timestep/friction sensitivity with held-out demonstrations.

Primary references: [official simulation setup](https://huggingface.co/docs/reachy_mini/en/platforms/simulation/get_started),
[pinned MJCF](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml),
[MuJoCo free joints](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-freejoint),
[contact model](https://mujoco.readthedocs.io/en/stable/computation/index.html#contact).
