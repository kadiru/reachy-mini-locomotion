# Training with published motor bounds — November 18, 2026

This experiment uses the new [motor model](MOTOR_LIMITS.md). It is simulation-only
parameter learning using the cross-entropy method (CEM), not reinforcement
learning and not a physical validation. Each policy has nine periodic head-motion
parameters; only the head motors provide locomotion. Body-yaw target is zero.

## Results

**The refined policy moves more slowly, with substantially less turning and
shaking than the initial trained policy.** It passes our simulated walking check
in 8/12 fresh held-out cases, versus 2/12 for the initial policy. All 12 refined
cases remain numerically valid and advance more than 1 mm. This is an improvement
in this model, not established real-world reliability.

| Nominal test | Initial trained | Refined |
|---|---:|---:|
| Measured cycles / duration | 24 / 11.04 s | 24 / 12.96 s |
| Head-sway frequency | 2.174 Hz | 1.852 Hz |
| Forward displacement, 1 ms physics | 64.0 mm | 32.1 mm |
| Sideways displacement magnitude | 109.5 mm | 5.1 mm |
| Heading change magnitude | 97.7° | 3.5° |
| RMS base acceleration (50 Hz samples) | 1.172 m/s² | 0.419 m/s² |
| Any head/body torque cap reached | 14.9% of measured steps | 0% |

The lower acceleration is about a **64% reduction**; this metric samples velocity
at 50 Hz and does not measure all high-frequency contact shocks. The refined
nominal run at 2 ms advances 34.0 mm, versus 32.1 mm at 1 ms (5.6% difference).
Its head-motor peak torque is 0.288 Nm, below the 0.416 Nm study cap. Actual head
roll spans about -34° to +36° and yaw -33° to +35°. Large instantaneous target
tracking error (~88°) remains because the motion has substantial phase lag.
It is optimized for this uncertain dynamics model, not a verified SDK replay.

Both foot edges unload in alternation, each for about 12% of measured time;
peak nominal foot tilt is 0.81° and opposite-rim clearance about 1.2 mm at 1 ms.
This is a subtle rocking shuffle. The whole foot is not expected to leave the
floor. Closure error remains below 0.029 mm in the rendered run.

Held-out speeds range 1.72–6.90 mm/s. Four cases fail the heading threshold
(11–15° turn); none are hidden. All seven refined domain pairs (nominal plus
six heldouts) pass the displacement timestep diagnostic. Heading changes between
timesteps still reach 4.6°. Robust closed-loop steering and measured motor/contact
identification are the next steps; no transfer-ready policy is claimed.

The user subsequently confirmed the reference video used **battery power**.
A separate constant-voltage sensitivity check holds everything else nominal:
5.0, 5.5 and 6.0 V, each at 2 and 1 ms, 24 cycles. All six pass the walking check;
forward travel is 32–48 mm. These are hypotheses within published motor data,
not measured battery/bus voltage or a sag/thermal simulation. The actual battery
is listed as 6.4 V nominal by Pollen; higher bus voltages are not extrapolated.

528 candidate evaluations produced 1,872 training rollouts; 98 separate
control/evaluation rollouts and six battery sensitivity rollouts were also run.
There were 25 invalid training rollouts in the initial study and 41 in refinement
(mainly IK-limit rejections); receipts retain their reasons. The rendered demo
is an additional deterministic 1 ms replay with saved states.

Artifacts:
- [1× playback, actual saved states](../media/simulation/motors-trained-2026-11-18.mp4)
- [Compact evaluation summary](../experiments/reports/motor-training-summary-final-2026-11-18.json)
- [Initial complete receipt](../experiments/reports/motor-training-2026-11-18.json)
- [Refined complete receipt](../experiments/reports/motor-training-refined-2026-11-18.json)
- [Battery sensitivity](../experiments/reports/motor-training-battery-2026-11-18.json)
- [Frozen gait parameters](../configs/sway-trained.json)

## Protocol

1. Initial search: seed 20261118, 10 generations × 24 candidates, three training
   domains, six measured cycles after two warm-up cycles. Objective rewards
   forward speed with lateral, heading, acceleration, torque and saturation
   penalties. Ranking mixes mean and worst-domain scores equally. It selected
   a moving but strongly turning gait. Initial implementation and receipt are
   preserved in commit `bf18fb5`.
2. Refinement: seed 20261119, 12 generations × 24 candidates, four training
   domains (including nominal at both 2 and 1 ms), ten measured cycles. Ranking
   is 80% worst-domain score and 20% mean. It penalizes sideways speed, heading
   rate and 50 Hz sampled base acceleration more heavily. It starts from the
   initial gait; every sampled candidate and failure is retained. Its best
   candidate was found in generation zero; subsequent generations did not improve
   that score. This is bounded parameter search, not evidence of RL convergence.
3. Freeze each winning policy before evaluation. Refined-policy evaluation uses
   six newly sampled domains (seed 67190), each at 2 and 1 ms, and 24 measured
   cycles. The first study's evaluation informed refinement and is therefore
   development evidence; the second study uses fresh holdouts. Nominal is a
   separate comparison, not counted as held out. Initial trained, historical
   untrained, and stationary controls are evaluated on the same domains.

All runs use CPU on the Mac, at most four workers. No shared GPU, robot client,
servo configuration, or physical robot was used. Resets use a fixed-foot solve
and settling, excluded from travel. Model hashes, engine version, parameters,
per-cycle displacement, tracking error, torque RMS/peaks, contact duty and failure
reasons are in the receipts. Samples at the same domain are deterministic;
12 holdout cases are six parameter sets × two timesteps, not 12 physical trials.

For the reporting check, a case qualifies only if numerically valid, speed is
at least 1 mm/s, lateral displacement is at most 30% of forward plus 2 mm,
absolute heading change is at most 10°, and peak base tilt is at most 12°.
These thresholds were set before the refinement holdout results were inspected.
They do not authorize hardware transfer. A separate timestep check allows
20% relative forward-displacement change plus 1 mm; it is a diagnostic,
not proof of solver convergence.

The geometry remains a cylinder approximation. Active and passive joint losses,
rotor inertia, PWM conversion, torque-speed curve and mass distribution are not
identified from this robot. In particular, large head tracking error exposes
model/trajectory lag; no thermal endurance or current-sensor accuracy is claimed.
A slowly progressing rocker may look more like a shuffle than distinct steps.

## Reproduce

Install the project sim extras and hydrate the pinned official model as described
in [CONTACT_PROBE.md](CONTACT_PROBE.md). Use new output paths; experiments refuse
to overwrite existing receipts.

```sh
.venv/bin/python experiments/train_sway.py --output experiments/reports/motor-training-new.json
.venv/bin/python experiments/refine_sway.py --output experiments/reports/motor-training-refined-new.json
.venv/bin/python experiments/summarize_motor_training.py experiments/reports/motor-training-refined-new.json --output experiments/reports/motor-training-summary-new.json
```

The refinement script explicitly uses the committed first-study policy as its
seed. The raw gait JSON alone omits the servo filter/latency and motor-model
configuration; it is not a hardware replay program.

To replay the frozen candidate and render saved states:

```sh
.venv/bin/python -m reachy_locomotion.sway --config configs/motors-datasheet.json --gait configs/sway-trained.json --cycles 24 --output experiments/generated/trained-new.json
.venv/bin/python experiments/render_sway.py experiments/generated/trained-new.json --output media/simulation/trained-new.mp4 --slowdown 1
.venv/bin/python experiments/battery_sensitivity.py --output experiments/reports/motor-training-battery-new.json
```

The checked-in config uses 2 ms physics. The displayed video uses the same config
with `timestep_s` set to 0.001; its embedded report retains that exact config.
MuJoCo graphics on macOS require a local graphics-enabled session. Rendering
replays saved qpos, never regenerates motion or invents base travel.
