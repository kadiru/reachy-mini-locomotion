# Heading feedback — November 19, 2026

**Outcome: better steering and retained forward speed, but the requested all-pass
stress result was NOT achieved.** The selected controller retains 89.2% of the
previous nominal speed. It reduces worst heading excursion on the original
stress set from 110.1° to 24.0°, but still passes 0/12 under the original 10°
criterion. These are uncalibrated MuJoCo results; no robot was commanded.

## Results

| Measurement | Previous fast gait | Selected feedback controller |
|---|---:|---:|
| Nominal 1 ms, 20.52 s: forward distance | 363.9 mm | 324.7 mm |
| Nominal speed | 17.74 mm/s | 15.83 mm/s |
| Nominal lateral displacement | -27.6 mm | -16.5 mm |
| Nominal net heading change after startup | -7.76° | -2.54° |
| Nominal maximum measured heading excursion | 9.03° | 5.89° |
| Original stress set, ≥12 s: full passes | 0/12 | **0/12** |
| Original stress set: mean forward speed | 12.08 mm/s | 18.62 mm/s |
| Original stress set: minimum forward speed | 4.29 mm/s | 13.90 mm/s |
| Original stress set: worst heading excursion | 110.07° | 24.03° |
| Fresh models/noise seeds: full passes | 2/20 | **4/20** |
| Fresh set: mean forward speed | 11.88 mm/s | 14.62 mm/s |
| Fresh set: worst heading excursion | 118.64° | 34.60° |

All selected-controller old/fresh trials are numerically valid and advance.
Validity is **not** the walking-success criterion. Fresh speed spans 2.11–24.48
mm/s; slow behavior remains in unseen models. At ≥20 s the original 12 stress
cases still have 0 passes, minimum speed 13.78 mm/s, mean 18.58 mm/s and maximum
heading excursion 26.30°. The all-pass/no-speed-loss objective remains open.

The shown nominal run has maximum startup-referenced heading error 5.62°, peak
foot tilt 3.07°, and actual joint-angle margin ≥0.2425 rad. Peak head torque is
0.3982 Nm under the unchanged 0.416 Nm nominal cap. Nominal 2 ms speed is 16.82
mm/s versus 15.83 at 1 ms (6.3% difference). Only 8/10 fresh model pairs satisfy
the previous coarse displacement check (difference ≤20% of larger distance
+1 mm); one differs markedly, 7.71 versus 20.87 mm/s. This is not converged,
physically calibrated transfer evidence.

[Nominal video, real-time playback](../media/simulation/heading-feedback-2026-11-19.mp4)
shows actual saved 1 ms states: 20.52 s measured plus 1.62 s startup. The
[predeclared first stress case](../media/simulation/heading-feedback-stress-2026-11-19.mp4)
advances 311.7 mm in 20.52 s, with 24.03° maximum measured turn: it is explicitly
a failed heading test, not a successful robustness demo. Both have fixed front/
side cameras, 5 cm grid, heading error, internal steering trim and sensor labels.

## What changed

The selected policy uses filtered PID heading feedback to add a bounded head-roll
bias, a smaller coupled yaw bias, and a small within-cycle timing skew. The main
large roll/yaw swing remains. Requested frequency 1.8285 Hz rounds to 1.85185 Hz
at 50 Hz control; roll amplitude is 38.01° and yaw amplitude 26.80° before bounded
corrections. Steering correction is capped at ±2.484° and 17.651°/s. Pose and IK
bounds, 50 Hz control and the original motor target filter/rate/torque model
remain in force. Body-yaw target remains zero. No base pose/velocity overwrite
or external push occurs during a rollout.

Feedback observes an **assumed external foot-heading estimate**, relative to
startup. Default simulation adds 60 ms delay and 0.25° Gaussian noise. It is not
a calibrated hardware estimator or a direct reading of the head IMU. The policy
has no access to randomized model parameters or contact forces. Its reference
is not reset when the measurement window starts. The original measured-heading
metric and an additional startup-referenced metric are retained separately.

The base model is unchanged: 1.475 kg total mass, nominal 5 V, head cap 0.416 Nm,
6 rad/s command speed and 60 rad/s² command acceleration. Stress voltage changes
use the same prior datasheet interpolation/caps (up to approximately 0.480 Nm at
6 V), not an increased nominal torque allowance. Mass placement, contact proxy,
servo response, motor-bus voltage under battery load and thermal behavior remain
unmeasured. See [motor sources and assumptions](MOTOR_LIMITS.md).

## Training and selection

This is parameter optimization and controller tuning, not reinforcement learning.
593 recorded candidate evaluations contain 4,151 completed training rollouts;
112 separate steering-probe rollouts examined fixed roll/yaw bias, phase skew
and twice-per-stride yaw corrections. At most four local CPU workers were used.
One search was stopped after 72 completed candidates because its feasibility-first
objective favored nearly stationary gaits; its partial receipt labels the stop,
and in-flight work is not counted as recorded evidence.

The formerly held-out seed 591191 is now development data. Searches tested raw
PID, cycle averages, stride-held updates, yaw-only steering, harmonic steering,
phase-based separation of rocking/drift, amplitude adaptation and gait blends.
Those variants remain available for reproducible experiments; **they are disabled
in the selected policy unless present in its frozen config**. The selected
feedback is the simpler filtered PID with roll/yaw/timing corrections.

Retained-speed ranking targets ≥14 mm/s nominal. Eight promising fast candidates
were checked for ≥12 s at 1/2 ms; three were extended to ≥20 s. Two candidates
nearly stopped in a stressed model at one timestep and were rejected from the
final speed-preserving selection. Final selection reads only those development
results, requiring nominal tests to pass at both timesteps, nominal speed ≥14
mm/s, and speed ≥8 mm/s in every old stress case. The chosen controller was
frozen before fresh evaluation; this rule does **not** assert that stress
heading tests pass.

Final fresh evaluation uses seed **2026111907**, ten new model configurations
at both timesteps, and independent noise seeds. It was not used to tune/select
the controller. The frozen development runs were reproduced exactly with the
final source before this evaluation. All failed trials remain in the receipts.

The original success criteria are unchanged: valid rollout, speed ≥0.5 mm/s,
maximum measured unwrapped heading excursion ≤10°, maximum lateral excursion
≤30% of forward displacement +3 mm, and foot tilt ≤18°. Heading and lateral
excursions are sampled at the 50 Hz control rate; tilt/contact/numerical checks
also run at each physics step. Endpoint heading alone cannot hide a full spin.

## Controls and limits

Disabling feedback while retaining the selected gait and identical startup
produces 81.60° worst heading excursion and 13.75 mm/s mean forward speed across
the old stress set, versus 24.03° and 18.62 mm/s with feedback. Thus the steering
improvement is not just a change of the nominal gait.

Additional sensor tests cover ideal readings, 100 ms/0.5° noise/+1.5° reference
bias, and 200 ms/1° noise/-2° reference bias, each on nominal plus two stress
models at both timesteps. Each group passes 2/6, the nominal cases; worst heading
excursions are 23.20°, 27.12° and 31.79°. Even ideal heading does not make this
controller satisfy all stress criteria. Steady heading correction, within-stride
yaw, lateral drift and contact/servo sensitivity remain unresolved together.

75 repository tests pass, including heading wrap, sensor delay/reproducibility,
anti-windup, bounded commands, optional estimator/adaptation behavior, and a
recorded physics rollout that preserves motor limits and reproducibility.

## Reproduce

Run from the repository root with its `.venv` and pinned/hydrated upstream model.
Use new output names; existing results are preserved.

```sh
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python experiments/refine_heading_gait.py --fast-feedback --generations 10 --population 28 --output experiments/reports/motor-training-heading-next.json
.venv/bin/python experiments/evaluate_heading.py --retain-speed --top 8 --search experiments/reports/motor-training-heading-next.json --heldout-seed 2026111908 --output experiments/reports/motor-training-heading-eval-next.json
```

The frozen video uses `configs/motors-datasheet.json` with `timestep_s` overridden
to 0.001, the two frozen policy files, 38 measurement cycles and three warmup
cycles. `experiments/record_heading_demo.py` reproduces the final nominal/stress
recordings from the final receipt; its dated outputs must not already exist.
For a new 2 ms default-model recording of the same controller:

```sh
.venv/bin/python -m reachy_locomotion.sway --config configs/motors-datasheet.json --gait configs/sway-heading-trained.json --feedback configs/heading-trained.json --sensor configs/heading-sensor-assumed.json --cycles 38 --warmup-cycles 3 --output experiments/generated/heading-next.json
.venv/bin/python experiments/render_sway.py experiments/generated/heading-next.json --output experiments/generated/heading-next.mp4 --slowdown 1 --title 'HEADING FEEDBACK' --diagnostics --frame-path
```

Receipts: `motor-training-heading-final-2026-11-19.json` contains the final
selection rule, source hashes, repeated development checks, ablations and fresh
model/sensor evaluations. `motor-training-heading-demo-2026-11-19.json` contains
video metrics and joint-limit checks. Full search receipts retain failed and
slower candidates. Raw saved states and preview GIFs remain local/ignored.
