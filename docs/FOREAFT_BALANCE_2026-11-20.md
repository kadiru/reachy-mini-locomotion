# Front/back edge balance — November 20, 2026

The new controller balances on either **front or back rim from an explicitly
raised starting posture**. The nominal 20-second simulations contain 19.76 s
of continuous front-edge hold and 19.54 s of back-edge hold. Natural
head-driven launch and capture from flat are still under development; a
raised reset does not demonstrate that transition.

Frozen hold policies: `configs/foreaft-balance-experimental.json`.
Earlier left/right balance and walking policies are preserved.

## Method and physical constraints

The front edge is the local +X rim, indicated by the green forward arrow in
the video; the back is −X. The controller uses pitch and pitch-rate feedback,
with lateral roll feedback to limit sideways departure. Commands and
observations are expressed in a frame rotated 90° about Z so the existing
balance controller can be reused. **The robot, its inertias, contacts and
gravity are not rotated or replaced.** Actual front/back dynamics are fitted
independently: reusing the front dynamics for the back failed.

Each side collects 64 two-second curriculum episodes beginning near
±32.1° pitch. The first 48 supply 760 filtered fitting transitions per side;
the remaining 16 are retained but are not scored as a held-out model test.
A local affine model includes body attitude, relative head pose,
their rates and recent commands; a quadratic regulator supplies feedback.
This is system identification and feedback-policy search, not neural RL.
The controller receives delayed simulated attitude and ideal head kinematics,
not contact forces, COM positions or hidden model parameters. Contact and COM
are used only for diagnostics and evaluation. Pose projection still calls
the physical inverse kinematics after the coordinate conversion.

The waist target stays exactly zero, held by its existing bounded servo.
The physical motor model, 10.7 rad/s target-speed limit, 360 rad/s² target
acceleration limit and 25 Hz target filter are unchanged. So are the actual
joint, closure, nonfoot-contact and 35° tilt guards. The 5 V supply, torque
derating, friction, mass distribution and sensor model remain uncalibrated
assumptions, not measured hardware behavior. No physical robot or shared GPU
was used.

An anchored diagnostic estimates neutral-head static tipping equilibria at
+31.90° front and −32.44° back. Head posture changes those values. This is
neither a universal falling angle nor a physical measurement.

## Acceptance and evidence

A hold requires the requested front/back rim to carry load, the opposite
half to be unloaded, pitch within 2° of its signed 32.1° target, pitch rate
at most 5°/s and lateral roll at most 5°. Qualification requires full trial
completion and all of the final five seconds meeting these criteria.
Flat recovery, a transient peak, an opposite-edge lift and a guard stop
cannot pass.

`motor-training-foreaft-hold-qualification-2026-11-20.json` records the frozen
hold-policy checks. The report distinguishes the artificially raised reset
from natural starts. Every recorded frame comes from actual MuJoCo state;
the renderer does not generate a desired trajectory or rerun the physics.

| Raised-start check | Front | Back |
| --- | --- | --- |
| Nominal, 20 s at 0.25 ms | Pass, 19.76 s hold | Pass, 19.54 s hold |
| 10 s at 0.125 ms | Pass | Pass |
| 10 s at 0.1 ms | Pass | Pass |
| Attitude noise 0.02° | 35° guard | 35° guard |
| Attitude noise 0.05° | 35° guard | 35° guard |
| Attitude delay 40 ms instead of 20 ms | Pass | 35° guard |
| Rolling friction zero | Pass | Pass |
| Total mass −10% | 35° guard | 35° guard |
| Total mass +10% | 35° guard | 35° guard |

That is **9/18** checks. Noise uses the fixed seed 202611224. Variants last
10 seconds unless noted; no retuning followed these checks. This is a small
sensitivity suite, not proof of generalization or hardware readiness.

Final nominal pitch is +31.658° front and −32.008° back; peak physical tilt is
32.10° and 34.54°. Maximum front/back rim-height differences are 59.52 and
63.43 mm. Peak head motor torques are 0.104 and 0.154 Nm, below the unchanged
0.416 Nm cap. Waist torque stays below 0.00245 Nm and deflection below 0.03°;
the waist is servo-held, not mechanically welded. Joint margins remain positive
and closure error below 0.013 mm. Both recordings use no workspace projection.

The feedback-disabled comparison retains the same artificial reset and
constant fitted equilibrium command, setting only the feedback gain to zero.
Its outcomes are stored in `motor-training-foreaft-ablation-2026-11-20.json`.
Without feedback, the front hits the tilt guard and the back returns flat;
neither sustains balance.
The repository suite passes **131 tests**, including front/back reset/contact
geometry, coordinate transforms and rejection of false hold evidence.

Video: `media/simulation/foreaft-balance-2026-11-20.mp4`, two independent
20-second raised-start trials at real speed. Both halves explicitly display
“ARTIFICIAL TILTED START”; they are not one continuous edge-to-edge transition.

Natural launch searches vary head pitch/translation, pumping frequency and
phase, cross-axis stabilization, predicted capture threshold, blending and
gain. The failed and completed attempts remain in the `motor-training-foreaft-*`
reports, including the initial mirrored-launch probe and subsequent pump,
catch, refinement and CEM searches. Stronger nodding can reach the tipping
region, but that alone is not a stable capture.
Across **289 natural-start search and diagnostic rollouts**, none qualifies;
the longest strict hold is just 0.04 s. These include repeated candidates,
not 289 independent policies. The final CEM search contains 144 front-edge
trials over four generations. Preserving the successful raised-start hold
must not obscure this unsolved transition.

## Replay

From the project root, with the existing local environment and upstream
simulation assets:

```sh
.venv/bin/python experiments/run_foreaft_balance.py configs/foreaft-balance-experimental.json --edge front --seconds 20 --timestep 0.00025 --diagnostic-raised-start --record --output experiments/generated/front-hold-replay.json
.venv/bin/python experiments/run_foreaft_balance.py configs/foreaft-balance-experimental.json --edge back --seconds 20 --timestep 0.00025 --diagnostic-raised-start --record --output experiments/generated/back-hold-replay.json
.venv/bin/python experiments/render_sway.py experiments/generated/front-hold-replay.json --output experiments/generated/front-hold-replay.mp4 --slowdown 1 --diagnostics --title 'FRONT EDGE - RAISED-START BALANCE'
.venv/bin/python -m unittest discover -s tests -v
```

Omitting `--diagnostic-raised-start` starts flat and tests the unqualified
launch recipe. Do not describe that mode as successfully trained.
For sensitivity checks, change `--timestep` or supply a copied `--config`;
sensor noise/delay live in each edge policy's `feedback` object.

Identification and fitting can be reproduced independently for each side
(`1` front, `-1` back):

```sh
.venv/bin/python experiments/train_foreaft_balance.py --identify --side 1 --count 64 --timestep 0.00025 --workers 6 --output experiments/generated/front-id-replay.json
.venv/bin/python experiments/fit_edge_regulator.py experiments/generated/front-id-replay.json --output experiments/generated/front-model-replay.json
.venv/bin/python - <<'PY'
import json
from pathlib import Path
policy = json.loads(Path('configs/foreaft-balance-experimental.json').read_text())['edges']['front']
policy['linear_gain_scale'] = 1.0
Path('experiments/generated/front-pump-seed.json').write_text(json.dumps(policy))
PY
.venv/bin/python experiments/train_edge_capture.py experiments/generated/front-pump-seed.json --foreaft --pump --side 1 --generations 4 --population 36 --seconds 12 --workers 6 --stop-on-hold --output experiments/generated/front-pump-search-replay.json
```

The pump seed is the front policy from the frozen configuration, with
`linear_gain_scale` set to 1.0 for the recorded search. Source hashes in
each report distinguish the exact implementation used for that experiment.

Next: widen the capture region and reduce the angular/head velocity at the
handoff, then qualify natural starts under noise, model variation and finer
physics. Longer holding alone does not resolve launch or sim-to-real transfer.
