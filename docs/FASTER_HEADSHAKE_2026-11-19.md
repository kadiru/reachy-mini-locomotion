# Larger swings and sharper braking — November 19, 2026

This experiment continues the [48.19 mm/s speed-first result](SPEED_FIRST_2026-11-19.md).
The user asked for more speed, a larger head swing and sharper braking, without
falling and with up to 20° heading change. All evidence here is uncalibrated
MuJoCo simulation; no physical robot was commanded.

**Result: 1.149 m in 20.02 s, or 57.39 mm/s — 19.09% faster**, with maximum
heading change 14.39°. [Watch the real-time video](../media/simulation/faster-headshake-2026-11-19.mp4).
The left camera follows the head; the right camera is fixed to show displacement
against a 5 cm grid. The video replays actual saved states at 1× (25 fps,
1280×580, 20.72 seconds including warmup).

## Sustained speed, turning and no-fall checks

| Evaluation | Speed | Forward travel | Maximum heading | Outcome |
|---|---:|---:|---:|---|
| Previous gait, 20.02 s, 1 ms | 48.19 mm/s | 0.965 m | 17.29° | Upright |
| Selected gait, 20.02 s, 1 ms | **57.39 mm/s** | **1.149 m** | **14.39°** | Upright |
| Selected gait, 20.02 s, 0.5 ms | 57.23 mm/s | 1.146 m | 13.47° | Upright |
| Selected gait, 20.02 s, 0.25 ms | 57.08 mm/s | 1.143 m | 13.25° | Upright |
| Selected gait, 40.04 s, 1 ms | 56.94 mm/s | 2.280 m | 15.18° | Upright |
| Selected gait, 120.12 s, 1 ms | **56.23 mm/s** | **6.754 m** | **15.18°** | Upright |

Distances exclude three warmup cycles and use the fixed startup forward axis.
Heading limits include startup. Lateral displacement in the video run is 23.5 mm.
The two-minute run has maximum foot tilt 2.43° and minimum motor joint-limit
margin 0.527 rad. Validity checks reject falls/35° tilt, nonfoot floor contact,
closure errors above 1 mm, invalid IK and numerical warnings; selection also
checks actual joint margins. These remain simulation checks, not guarantees on
the physical robot.

After freezing, eight new randomized models were each tested at 1 and 0.5 ms
(seed 2026111923), varying friction, voltage, motor gain, latency, passive/active
joint properties, foot radius and mass distribution. **16/16 stay upright and
move forward, but only 6/16 stay within 20°**. Worst heading is 114.43°; speeds
range from 2.56 to 95.58 mm/s. The two timesteps are not independent model samples.
This is a different seed from the earlier experiment, so these totals do not
establish improved robustness. Steering under model variation remains unresolved.

## Larger swing and braking evidence

The yaw command amplitude rises from 37.15° to **40.44°**, while roll amplitude
decreases from 38.96° to 37.91°. The main roll/yaw phase offset changes from
120.11° to 110.13°, with a small yaw harmonic and translation/pitch adjustments.
The actual frequency remains **4.545 Hz** (11 control frames per cycle), and the
existing 10° heading-deadband feedback is retained. The selected waveform uses
the smooth swing family; the explicit trapezoidal brake/hold candidates did not
win. Their inactive stroke-fraction parameters do not affect the selected motion.

Measured head yaw spans **55.88°**, up from 50.62° (+10.4%), and its peak observed
yaw rate rises from 685 to 731°/s. Measured roll span decreases from 25.46° to
19.98°. Single-edge support occurs in 99.8% of sampled frames, and 99.6% of
observed yaw-slowing frames coincide with single-edge support. **No airborne
interval occurs** under the existing foot-clearance/normal-force diagnostic.

Matched ten-second ablations retain the other selected parameters:

| Change | Speed | Maximum heading |
|---|---:|---:|
| Selected policy | 57.47 mm/s | 12.43° |
| Restore previous yaw amplitude | 55.35 mm/s | 17.70° |
| Restore previous waveform sharpness | 57.46 mm/s | 13.17° |
| Restore previous phase offset/skew | 40.52 mm/s | 51.68° |
| Remove the small yaw harmonic | 57.38 mm/s | 8.49° |

Timing has the larger effect here. Restoring the old sharpness changes speed
very little, so the gain should not be attributed to sharper braking alone.
Average negative motor mechanical work per second actually decreases from
0.655 to 0.634 W. These are simulated dynamics, and sampled phase correlations
do not independently establish causality.

## Fixed limits and search protocol

`configs/motors-speed-trained.json` is unchanged, including both physical and
software assumptions: 5 V nominal hypothesis, 1.475 kg mass, 0.416 Nm head torque
caps, 80% PWM, manufacturer no-load-speed/back-EMF envelope, target speed
10.7 rad/s, acceleration 360 rad/s², 25 Hz target filter and one-frame delay.
Joint limits, the contact model and body yaw commands are unchanged. The actual
battery motor-bus voltage and installed firmware response remain unmeasured.
See [motor sources and limitations](MOTOR_LIMITS.md).

`experiments/train_faster.py` first probes larger yaw amplitudes, sharper
waveforms, phases and finite-braking strokes, then optimizes 22 gait parameters.
The search uses 1 ms physics, eight-second whole-cycle evaluations and four CPU
workers. Forward progress is measured along the fixed startup heading. There
is no smoothness or lateral-motion penalty. Candidates must move forward, remain
valid/upright and stay within 20°; unsuccessful exploration is retained in the
reports. Rejected joint-limit poses are not executed or silently clipped by IK.

`experiments/probe_braking.py` independently tests slower, wider swings and short
braking phases to explore beyond the previous gait's frequency range.
`experiments/select_faster.py` evaluates leading candidates with three bounded
heading-feedback variants over 20 seconds, checks 0.5 ms physics and 40-second
endurance, then freezes the winner before fresh randomized model trials.
The final recorded replay must numerically match its selection evaluation.

There are **693 recorded candidate evaluations**: 65 initial probes, 280 from
seven complete generations of the interrupted search, 108 broad braking probes,
and 240 from six recovered/refined generations. Unrecorded work in the interrupted
generation is not counted. Selection uses 55 twenty-second runs, eight 0.5 ms
checks and four 40-second runs. The fastest conservative sustained speed across
these checks selects `0-same`. Fresh stress trials follow freezing, then the
exact recorded replay, five diagnostic ablations and two additional validation
runs. Training uses seeds 2026111921/2026111924; broad probes use 2026111922.

## Numerical boundary repair

The first refinement search stopped after seven completed generations because
steering's amplitude clip could add back to `60.00000000000001` degrees when yaw
amplitude, bias and a harmonic were summed. That arithmetic roundoff raised the
pose validator's strict `>60` check. The incomplete report preserves all completed
generations; a subsequent search explicitly recovers those seeds.

The validator now tolerates only 1e-12 degrees of arithmetic roundoff. Motor and
IK limits are unchanged. A regression test reproduces the clipped harmonic case
and verifies that 0.000001° excess is still rejected. This changes acceptance of
roundoff cases, not the dynamics of previously valid trajectories. Original
launch sources are retained locally by their report hashes.

## Reproduction

Use the project environment and pinned model from [CONTACT_PROBE.md](CONTACT_PROBE.md).
The searches run on local CPUs; no shared GPU work is involved. The dated reports
and video are preserved. Choose new output paths when repeating experiments.

```sh
.venv/bin/python -m reachy_locomotion.sway \
  --config configs/motors-speed-trained.json \
  --gait configs/sway-faster-trained.json \
  --feedback configs/heading-faster-trained.json \
  --sensor configs/heading-sensor-assumed.json \
  --warmup-cycles 3 --cycles 91 \
  --output experiments/generated/faster-replay-next.json
```

For the video's fixed-start-axis progress, use
`measurement_displacement_in_startup_frame_m[0]`. The legacy `forward_m` field
uses the heading when measurement starts. The recorded video result sets
`render_forward_frame` to `startup`; the renderer reads that field.

```sh
.venv/bin/python experiments/render_sway.py \
  experiments/generated/faster-20s-2026-11-19.json \
  --output media/simulation/faster-headshake-next.mp4 \
  --slowdown 1 --title 'FASTER HEADSHAKE' \
  --diagnostics --frame-path --follow-front
```

The saved trajectory is generated locally by `experiments/select_faster.py`;
it is not committed. That selector takes completed reports using `--input`, a
new `--output` report and new `--recording` path, and writes the two frozen
gait/heading configs. Preserve the current configs before rerunning selection.
`experiments/analyze_faster.py` consumes selection/recording paths and a new output
path to reproduce the ablations and additional validation.

Evidence in `experiments/reports/`:

- `motor-training-faster-{probe,braking,search,refined}-2026-11-19.json`: candidates and launch receipts; `search` is explicitly incomplete.
- `motor-training-faster-interruption-2026-11-19.json`: recovery and preserved-report hash.
- `motor-training-faster-selection-2026-11-19.json`: long/fine trials, frozen policy, fresh stress trials and exact replay verification.
- `motor-training-faster-summary-2026-11-19.json`: comparisons, ablations and the two-minute/0.25 ms checks.
- `motor-training-faster-delivery-2026-11-19.json`: exact CLI replay, unchanged motor-config verification, video metadata and final source/media/report hashes.
- `repository-tests-faster-2026-11-19.txt`: **81 tests passed**.

The motor/contact model remains unidentified, with no thermal, supply-sag or
backlash calibration. The next step is steering robustness while preserving this
faster baseline, followed by actual motor/profile/contact identification before
sim-to-real trials.
