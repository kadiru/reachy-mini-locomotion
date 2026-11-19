# Large rocking steps with cycle-end heading limits

The user confirmed that the 20° heading allowance applies **after every complete
left/right cycle**, with larger torso swings permitted within the step. The
reference stays fixed at startup; it is not reset each cycle. This experiment
trains the separate large-rocking style while preserving the rapid-shuffle policy.
All runs are uncalibrated simulation, with no physical robot or shared GPU access.

## Result: a larger-rocking demonstration, not continuous qualification

[Real-time video](../media/simulation/rocking-steps-2026-11-19.mp4): the frozen
experimental gait advances **1.004 m in 20.16 measured seconds**, with no fall,
at **49.81 mm/s**. It makes **15.94 mm per complete left/right cycle** at 3.125 Hz,
26.2% longer cycles of forward progress than the preserved rapid mode. Median
roll reaches at least **6.14° on each side**, peak total body tilt is **8.56°**,
and both edges load within every measured cycle. The rapid mode remains faster
at 57.39 mm/s, with 12.63 mm/cycle and 2.43° maximum body tilt.

The video is the **best twenty-second, two-timestep demonstration** from the
tested recipes. It is not the winner of the original full acceptance protocol:
both long-run selection attempts failed. The frozen configs are deliberately
named `configs/{sway,heading}-rocking-experimental.json`.

| Twenty-second check | Speed (mm/s) | Progress per cycle (mm) | Maximum cycle-end heading | Completion |
|---|---:|---:|---:|---|
| 1 ms, recorded video | 49.81 | 15.94 | 11.57° | Upright, valid |
| 0.5 ms | 45.78 | 14.65 | 15.18° | Upright, valid |
| 0.25 ms | 45.01 | 14.40 | 15.66° | Upright, valid |

The 1 ms result differs from the 0.5 ms result by 8.1% using the selection
measure; 0.5-to-0.25 ms speed changes by 1.7%. Both fine checks miss the original
15 mm stride target. At 0.25 ms the instantaneous torso heading reaches 23.67°,
which is allowed by the user's cycle-end interpretation. No heading reference
resets or favorable half-cycle accounting are used.

The nominal trial has 60.1 mm lateral displacement, 8.97° mean within-cycle yaw
excursion, and a 16.22 mm maximum side-rim height proxy. Every measured cycle
has positive startup-forward progress; the smallest is 1.23 mm. There are **no
airborne intervals**: this is edge-supported rocking, not an airborne leap.
Minimum actual motor joint-limit margin is 0.436 rad and head peak torque stays
at or below the existing 0.416 Nm cap (up to floating-point roundoff).

### Failures that prevent a reliability claim

- In the 40-second test, the selected controller exceeds 20° cycle-end heading
  and stops on an unreachable head target after **38.04 measured seconds**.
  Requesting 120 seconds reproduces the same stop. The guard does not execute
  the invalid target, and no fall precedes the stop.
- A second twenty-second survivor stops after 22.36 seconds. Thus neither
  original nor provisional long-run selection succeeds.
- Twenty-two additional bounded-steering recipes all complete forty seconds
  upright at both 1 and 0.5 ms, but **none** meets the heading bound at both.
  Smaller trims solve the joint-limit-stop problem for these variants while
  losing directional control. They do not replace the frozen short demo.
- **Fresh variations: 8/16 valid completions, 0/16 course passes.** Eight stop
  at the IK joint-limit guard; the eight complete runs stay upright but turn too
  much. No fall/nonfoot-contact termination is observed; partial guarded runs
  are not counted as successful no-fall completions. Worst accumulated cycle-end
  heading is 357.85°. These eight randomized models at two timesteps use seed
  2026111933, after the demonstration recipe was frozen.
- Mirroring the gait completes but turns 61.84°. Replacing its yaw stroke with
  a sine wave stops at the IK guard after 13.4 seconds and has less bilateral
  rocking. This supports retaining the learned compound waveform, but does not
  isolate braking torque as the sole cause of forward motion.

The motor/contact model and assumed external heading sensor remain uncalibrated.
The software profile is not an installed firmware measurement. No physical
walking or safe continuous-duty operation is established.

Evidence: `motor-training-rocking-demonstration-2026-11-19.json` and
`motor-training-rocking-summary-2026-11-19.json` under `experiments/reports/`.
The saved replay exactly matches the selection rollout. The video contains 529
frames at 25 fps, 1280×580, 21.16 seconds including startup, played at 1×. The
front camera follows the body; the side camera is fixed. Early, late and final
frames were visually checked, including the +1004.2 mm final displacement.

## Objective and constraints

The physical and software motor configuration is the existing
`configs/motors-speed-trained.json`: nominal 5 V hypothesis, 1.475 kg total mass,
0.416 Nm head torque caps, the manufacturer no-load-speed/back-EMF envelope,
10.7 rad/s target speed, 360 rad/s² target acceleration, 25 Hz target filter and
one-frame delay. Joint/IK limits, friction/contact model and zero body-yaw
commands are unchanged. These are the same explicitly uncalibrated assumptions
described in [MOTOR_LIMITS.md](MOTOR_LIMITS.md).

The simulation still rejects tilt beyond 35°, nonfoot floor contact, excessive
closed-chain error, invalid IK and numerical warnings. Selection checks actual
motor joint margins too. Permitting intermediate yaw excursions does not relax
these constraints or allow falls.

The new `CycleMonitor` records signed progress along the fixed startup forward
axis, roll to both sides, support occupancy, yaw excursion and cycle-end heading.
It observes the 50 Hz state without changing the dynamics. Raw within-step yaw
and substep maximum body tilt remain visible as separate diagnostics. The heading
test covers every completed cycle, including the warmup cycles.

The predeclared style targets are:

- At least 6° median roll to **each** side across measured cycles; one large lean
  or a startup transient cannot satisfy this.
- At least 15 mm mean signed forward travel per complete left/right cycle.
- Both edges each carry load for at least 10% of a cycle in at least 80% of cycles.
- Positive sustained progress, valid/no-fall completion and no cycle-end heading
  outside ±20° relative to startup.

The 6°/15 mm values distinguish this style from the existing rapid gait's 2.43°
maximum tilt and 12.63 mm/cycle. They are experimental style targets, not claimed
manufacturer or hardware safety limits. Backward portions remain in signed
progress; rocking in place does not receive displacement reward.

## Cycle-end feedback and search

`HeadingFeedback.cycle_endpoint` reads only delayed measurements corresponding
to the same phase after a full cycle. The known simulated sensor delay determines
when that sample arrives. Bounded PI corrections are held between cycle updates
and applied through the existing head-target mapping and slew limit. It uses
the same assumed external heading estimator (60 ms delay, 0.25° noise), with no
contact-force or privileged model observations in the policy. Physical sensing
and motor response remain unvalidated.

Open-loop and cycle-end-feedback seeds are compared. The search explores larger
roll/yaw, lateral head translation, phase, finite turn/brake/hold strokes and pitch
timing. Initial search scoring rewards reaching a bilateral-rocking region; the
subsequent `--progress-focus` stage stops rewarding extra tilt above 6° and
concentrates on longer strides. The hard style/heading/no-fall gates are unchanged
between stages. Source receipts preserve the original first-stage script; the
later CLI adds multiple seed reports and the explicit curriculum switch.

Leading candidates must then survive longer runs and a smaller physics timestep.
Fresh model variations are evaluated after policy selection, not used to choose
the winner. The saved video replay must numerically match its selection rollout.

## Validation of the implementation

`tests/test_rocking.py` checks that within-cycle yaw can exceed 20° while every
endpoint is checked; endpoint drift still fails. It also checks signed progress,
bilateral support, sensor-delay alignment and rejection of falls/small shuffles.
All 85 tests passed after the new metrics/controller were added; the existing
physical motor-cap and recorded-versus-unrecorded invariance tests still pass.
The development log is `experiments/reports/repository-tests-rocking-development-2026-11-19.txt`.

## Search receipts

The completed training reports record 1,441 candidate evaluations: 129 initial
probes, 400 initial search candidates, 176 turn/brake/pitch probes, 400
progress-focused candidates, and 16 longer seed checks plus 320 local refinement
candidates. Initial probes/search use eight-second whole-cycle runs; refinement
uses twelve seconds. All use 1 ms physics and four local CPU workers.

Reports are `experiments/reports/motor-training-rocking-{probe,search,steps,progress,refined}-2026-11-19.json`.
Failed candidates are retained. Search seeds and source hashes are recorded in
each receipt. `--progress-focus` changes exploration ranking, not the physical
model or final acceptance gates. The speed of the existing rapid mode remains a
separate baseline; this search prioritizes longer individual rocking cycles.

For a fresh refinement, use a new report filename:

```sh
.venv/bin/python experiments/train_rocking.py --mode search \
  --input experiments/reports/motor-training-rocking-progress-2026-11-19.json \
  --progress-focus --local-scale .5 --seconds 12 --seed 2026111934 \
  --generations 8 --population 40 \
  --output experiments/generated/rocking-refined-next.json
```

Selection compares open-loop and bounded heading-feedback variants, then checks
the leading twenty-second runs at 0.5 ms and forty seconds. The frozen policy is
evaluated on sixteen fresh trials (eight randomized models, each at two physics
timesteps); those trials do not choose the winner. Independent checks extend
nominal duration to 120 seconds, reduce the timestep to 0.25 ms, and compare
mirrored motion and a sinusoidal-yaw ablation.

```sh
.venv/bin/python experiments/select_rocking.py \
  --input experiments/reports/motor-training-rocking-refined-2026-11-19.json \
    experiments/reports/motor-training-rocking-progress-2026-11-19.json \
  --output experiments/generated/rocking-selection-next.json \
  --recording experiments/generated/rocking-20s-next.json
.venv/bin/python experiments/verify_rocking.py \
  --selection experiments/generated/rocking-selection-next.json \
  --output experiments/generated/rocking-summary-next.json
```

Selection writes the dedicated rocking gait/feedback configurations. It never
overwrites the rapid mode or changes the shared motor configuration.

## First selection failure and paired refinement

The initial selection report is intentionally incomplete: none of its candidates
passed all long/fine style gates. Its best open-loop twenty-second candidate
made 16.04 mm/cycle, but at 0.5 ms its cycle-end heading reached 58.52°.
The best steering near-miss (`6-yaw_only`) stayed upright and within 20° at both
timesteps, with 15.94/14.65 mm per cycle at 1/0.5 ms and at least 6.14° bilateral
rocking. These are development results, not a successful final acceptance claim.
The 15 mm style threshold was not silently relaxed.

`experiments/refine_rocking_feedback.py` starts from that steered near-miss and
jointly refines gait and yaw-bias feedback. Each candidate is evaluated for twenty
seconds at both 1 and 0.5 ms; ranking uses the weaker stride/rocking and the worse
cycle-end heading. Invalid 1 ms candidates can skip the fine evaluation. All
physical limits and style gates stay fixed. The first selection's fine tests
are now development evidence, not untouched holdouts.

`experiments/probe_rocking_coordinates.py` separates steering and individual
motion changes, including frequency and braking duration. Recipe-mode selection
keeps learned gait/feedback pairs together, instead of substituting a generic
controller after training.

Neither the paired refinement nor the coordinate probe met all original style
gates at both timesteps. A separate, explicitly **provisional** selection uses
`--allow-style-near-miss`: it retains the original feasibility flags and requires
upright/valid completion, every cycle-end heading within 20°, at least 6°
bilateral roll, the same support-transfer gate, and greater stride than the
12.63 mm rapid-mode baseline. Thus a 14.65 mm fine-timestep stride remains a
reported miss of the original 15 mm target. This exception changes only the
experimental stride target; it does not relax heading, motor, joint or fall
checks. The original failed selection report remains intact.

```sh
.venv/bin/python experiments/refine_rocking_feedback.py \
  --input experiments/reports/motor-training-rocking-selection-2026-11-19.json \
  --output experiments/generated/rocking-paired-next.json
.venv/bin/python experiments/probe_rocking_coordinates.py \
  --input experiments/generated/rocking-paired-next.json \
  --output experiments/generated/rocking-coordinate-next.json
.venv/bin/python experiments/select_rocking.py \
  --input experiments/generated/rocking-paired-next.json \
    experiments/generated/rocking-coordinate-next.json \
  --recipe-mode --allow-style-near-miss \
  --output experiments/generated/rocking-provisional-next.json \
  --recording experiments/generated/rocking-20s-provisional-next.json
```

The provisional long-run selection also failed; it must not be treated as a
successful output. `record_rocking_demonstration.py` explicitly freezes only
the strongest twenty-second survivor from that saved failed selection, checks
the recorded states against the selection run, then runs the fresh suite.
Its report says `long_horizon_qualified: false`.

The complete search counts are 1,611 evaluated recipes / **1,719 simulated
rollouts**, including the later paired/coordinate/steering work. Qualification
adds 110 rollouts, followed by 16 fresh trials, four frozen-policy checks and
two exact recording/CLI replays. Unit-test simulations are separate. The count
breakdown, file hashes, unchanged motor/rapid-policy checks and video metadata
are in `motor-training-rocking-artifacts-2026-11-19.json`.

## Reproduce the delivered demonstration

Use the project-local environment and pinned model described in
[CONTACT_PROBE.md](CONTACT_PROBE.md). Choose unused output paths.

```sh
.venv/bin/python -m reachy_locomotion.sway \
  --config configs/motors-speed-trained.json \
  --gait configs/sway-rocking-experimental.json \
  --feedback configs/heading-rocking-experimental.json \
  --sensor configs/heading-sensor-assumed.json \
  --warmup-cycles 3 --cycles 63 \
  --output experiments/generated/rocking-cli-next.json
```

For the exact annotated video recording and fresh-model receipt:

```sh
.venv/bin/python experiments/record_rocking_demonstration.py \
  --selection experiments/reports/motor-training-rocking-selection-provisional-2026-11-19.json \
  --output experiments/generated/rocking-demonstration-next.json \
  --recording experiments/generated/rocking-video-next.json
.venv/bin/python experiments/verify_rocking.py \
  --selection experiments/generated/rocking-demonstration-next.json \
  --output experiments/generated/rocking-verification-next.json
.venv/bin/python experiments/render_sway.py \
  experiments/generated/rocking-video-next.json \
  --output media/simulation/rocking-steps-next.mp4 \
  --slowdown 1 --title 'EXPERIMENTAL ROCKING' \
  --diagnostics --frame-path --follow-front
```

The renderer consumes saved states without rerunning physics. Its cycle-end
heading label uses the original startup reference and stays distinct from the
instantaneous torso yaw and body-roll labels. The demonstration report's
`completed` field means its bounded evaluation/recording pipeline finished;
it does not override the explicit failed long-horizon qualification.
