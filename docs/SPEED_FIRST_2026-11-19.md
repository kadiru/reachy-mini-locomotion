# Speed-first compound headshake — November 19, 2026

**Uncalibrated MuJoCo simulation only.** The selected gait moves **0.965 m in
20.02 s**, or **48.19 mm/s**, with maximum heading change **17.29°**. This is
**2.73×** the previous fast gait under the same physical assumptions. The user
prioritized speed and remaining upright, allowing up to 20° heading change.
There is no smoothness or lateral-displacement penalty.

[Watch the evaluated run at real-time speed](../media/simulation/speed-first-2026-11-19.mp4).
The video is 1280×580, 25 fps, 20.72 seconds including startup, with fixed cameras,
a 5 cm grid and measured displacement/heading. It replays saved simulation
states; recording exactly reproduces the selected evaluation.

## Sustained motion and comparison

Forward speed uses displacement along the fixed heading at startup, divided by
the measurement duration. It is not path length. Distances exclude three warmup
cycles; heading constraints include startup. Durations cover whole 50 Hz cycles.

| Evaluation | Forward speed | Maximum heading | Outcome |
|---|---:|---:|---|
| Previous fast gait, approximately 20 s, 1 ms physics | 17.66 mm/s | See selection report | Baseline |
| Previous heading-feedback gait, approximately 20 s, 1 ms | 15.82 mm/s | See selection report | Baseline |
| Selected gait, 20.02 s, 1 ms | **48.19 mm/s** | **17.29°** | Upright; 964.8 mm forward, 97.2 mm lateral |
| Selected gait, 20.02 s, 0.5 ms | **48.47 mm/s** | **16.62°** | Upright; 0.57% speed difference |
| Selected gait, 40.04 s, 1 ms | **48.25 mm/s** | **17.29°** | Upright; 1.93 m forward |

The 40-second run has maximum foot tilt 3.22° and minimum motor joint-limit margin
0.468 rad. No external forces or base position/velocity writes propel the robot.
Validity requires no nonfoot floor contact, tilt below 35°, closed-chain error
below 1 mm, no numerical warnings and valid IK; selection also checks actual
motor joint limits. These are simulation checks, not a guarantee against falling
on hardware.

The earliest, visually promising demo used 1.2 Nm head actuators and lower CAD
mass. It remains [historical evidence](FORWARD_PROGRESS_COMPARISON_2026-11-19.md),
not the comparison used for the 2.73× gain.

## What enabled the gain

Physical assumptions are unchanged: nominal 5 V (unmeasured battery motor bus),
1.475 kg mass, 80% PWM/torque limits, **0.416 Nm head torque caps**, manufacturer
back-EMF/no-load-speed envelope, joint limits and contact model. See
[motor sources and limitations](MOTOR_LIMITS.md). Body yaw commands remain zero.

The old target speed, acceleration and smoothing were conservative software
choices, not manufacturer limits. The search compared these command profiles:

| Software setting | Previous profile | Selected profile |
|---|---:|---:|
| Target speed | 6 rad/s | 10.7 rad/s |
| Target acceleration | 60 rad/s² | 360 rad/s² |
| Target filter | 5 Hz | 25 Hz |

Targets still pass through 50 Hz sampling, delay, IK and bounded motor dynamics.
The target-speed ceiling is also capped by each motor's no-load speed. Actual
back-driving is not artificially clipped. No firmware setting was changed, and
the selected command profile has not been verified against the installed robot.

The requested gait frequency is 4.647 Hz; sampling produces **4.545 Hz** (11
control frames/cycle). It combines approximately 39° commanded roll amplitude
and 37° commanded yaw amplitude with a 120° phase offset. Motor tracking does
not attain those ideal commands: observed head roll spans −20.1° to +5.4°, and
yaw −17.9° to +32.7°. The heading controller reacts to filtered errors outside a
10° deadband, using the existing assumed 60 ms delay and 0.25° noise model.

We added and searched explicit trapezoidal-speed strokes with a short brake and
endpoint hold. The winning candidate instead uses the existing smooth swing
family with faster command timing and a high-frequency compound roll/yaw motion.

## Braking, edge support and the proposed leap

Mean progress is **10.60 mm per cycle** (7.43–11.76 mm). Single-edge support occurs
in 99.6% of 50 Hz samples; 99.2% of detected head-slowing samples coincide with
single-edge support. Negative motor mechanical power is observed. This supports
the plausibility of the proposed interaction, but phase correlation does not
prove that braking alone causes forward movement.

**No fully airborne flight was detected.** The flight diagnostic requires the
lowest point of the foot cylinder to clear the floor by more than 0.1 mm and
summed foot normal force to fall below 0.05 N, checked at physics substeps.
Raising one rim while the opposite edge stays supported is not counted as flight.
The result is fast edge-supported rocking steps.

Ten-second ablations keep the selected gait fixed; these are diagnostics, not
separately optimized policies:

| Change | Forward speed | Maximum heading |
|---|---:|---:|
| Selected policy | 48.53 mm/s | 17.29° |
| Restore 60 rad/s² target acceleration | Approximately 0 | 0.43° |
| Restore 5 Hz filter | 5.54 mm/s | 101.59° |
| Restore 6 rad/s target speed | −0.11 mm/s | 0.55° |
| Restore all previous command settings | Approximately 0 | 0.30° |
| Disable heading feedback | 48.35 mm/s | 18.60° |
| Remove dynamic yaw, open loop | 9.71 mm/s | 75.11° |
| Remove dynamic roll, open loop | 0.065 mm/s | 0.37° |

All ablations stay upright. They show that coupled head motion and command timing
matter for this gait. Changing command timing also changes amplitude and phase;
it does not isolate a braking-only mechanism.

## Selection and fresh stress tests

Two bounded CPU searches evaluated **612 candidates** (8 generations × 36 plus
18 initial candidates per search), seeded by previous gaits and explicit stroke
variants. Seeds were 2026111911 and 2026111912. The second search retained a basin
around the original high-frequency gait. There were also 24 baseline rollouts.

Selection used 84 development rollouts at 20 seconds and 2/1 ms, eight further
0.5 ms checks, and four 40-second checks. Acceptance required positive forward
movement, validity and heading within 20°; 1/0.5 ms speed agreement had to be
within 15%. The fastest survivor became frozen policy `11-deadband10`.

Only after freezing, eight fresh randomized models were each evaluated at 1 and
0.5 ms (seed 2026111913). They vary voltage 4.7–6 V, friction, gain, latency,
passive/active joint properties, foot radius and mass distribution.

**All 16 stay upright and move forward; only 6/16 meet the 20° heading limit.**
Worst heading is 84.15° and forward speeds range from 1.55 to 73.98 mm/s.
These are eight model samples evaluated twice, not 16 independent physical
experiments. Robust steering is unresolved, and performance remains sensitive
to unidentified physical parameters. No thermal, backlash or battery-sag
validation has been performed. No real-robot execution occurred.

## Reproduce and inspect

Use the project environment and pinned official model described in
[CONTACT_PROBE.md](CONTACT_PROBE.md). Run from the repository root, choosing a
new output filename:

```sh
.venv/bin/python -m reachy_locomotion.sway \
  --config configs/motors-speed-trained.json \
  --gait configs/sway-speed-trained.json \
  --feedback configs/heading-speed-trained.json \
  --sensor configs/heading-sensor-assumed.json \
  --warmup-cycles 3 --cycles 91 \
  --output experiments/generated/speed-replay-next.json
```

The frozen config uses 1 ms physics. For fixed-start-axis forward displacement,
read `measurement_displacement_in_startup_frame_m[0]`; the legacy `forward_m`
field uses the heading at measurement start. The saved video trajectory sets
`render_forward_frame` to `startup` so its overlay uses the selection metric.

Search reproduction, with new report paths:

```sh
.venv/bin/python experiments/train_speed.py --profile existing \
  --seed 2026111911 --output experiments/generated/speed-existing-next.json
.venv/bin/python experiments/train_speed.py --profile sharp_targets \
  --seed 2026111912 --start-gait configs/sway-candidate.json \
  --output experiments/generated/speed-sharp-next.json
```

`experiments/select_speed.py --existing <report> --sharp <report> --output <new-report>`
performs the long/fine selection and fresh stress checks; it writes the three
frozen policy configs. Preserve the current configs before running a new search
selection. `experiments/record_speed_demo.py` produced the exact replay and
ablations; its dated outputs intentionally refuse overwrite.

The first search ran before the optional `--start-gait` argument was added;
the default behavior is unchanged. The original source was reconstructed and
hash-verified against its launch receipt and retained locally. Reports retain
launch source hashes; the summary also identifies final reviewed sources/media.

Evidence under `experiments/reports/`:

- `motor-training-speed-summary-2026-11-19.json`: compact metrics, hashes and video metadata.
- `motor-training-speed-{baselines,existing,sharp,selection,demo}-2026-11-19.json`: full numerical evidence.
- `repository-tests-speed-final-2026-11-19.txt`: **80 tests passed**, including physical caps, replay invariance, true-flight discrimination and heading deadband behavior.

Next work is steering robustness while retaining this faster gait, followed by
identification of real motor/profile/contact response before any sim-to-real trial.
