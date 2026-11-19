# Two head-driven walking styles

The user's November 19 clarification defines two desired gait families. Keep a
separate best policy for each, rather than selecting every experiment solely by
average speed. Both must respect the same motor/joint limits and remain upright.
This document records the intended behavior and evaluation changes. A larger
rocking demonstration is now implemented in simulation; continuous qualification
and a mode-switching controller remain incomplete.

| Style | Intended motion | Main selection goal | Current evidence |
|---|---|---|---|
| Rapid small steps | Fast alternating head motion; small body rocking and frequent support changes | Sustained forward speed with no falls | Existing 57.39 mm/s nominal simulation |
| Large alternating-edge steps | Larger sway, head turn and timed braking; pronounced torso roll/yaw, lifting the opposite side before transferring support | Larger signed forward progress per complete left/right cycle, with actual support transfer and no falls; retain speed as a separate metric | Experimental 20-second simulation: 49.81 mm/s, 15.94 mm/cycle, 8.56° peak body tilt; longer-run steering fails qualification |

## Large-step mechanism to test

The proposed sequence is:

1. Load one edge of the base while lifting the opposite side.
2. Coordinate head roll and yaw, then brake the head while that edge carries load.
3. Use the resulting whole-body motion to advance the lifted side, then land it.
4. Transfer support and mirror the motion for the other side.

For a pivot without slip, the loaded contact anchors that part of the base while
the opposite side sweeps forward. Sliding or rolling of the loaded contact may
also contribute; the actual finite contact patch must be measured. Internal head
motion alone is not an external propulsive force—the ground reactions and contact
sequence matter. A fully airborne hop is not required for this style.

This is a testable interpretation of the user's description. The supplied footage
supports coupled head/torso motion and apparent progress, but does not resolve
exact contact forces, a stationary pivot, head braking torque or metric steps.
No physical tilt angle or safe tilt envelope is inferred from pixels.

## Why the current fastest gait is only one style

Analysis of the saved `faster-20s-2026-11-19.json` trajectory gives:

- 4.545 complete left/right cycles per second.
- 12.63 mm mean forward progress per full cycle, across 91 measured cycles.
- 2.43° maximum body tilt; body roll ranges from −1.86° to +2.42°.
- 4.33 mm maximum side-rim height from the existing geometric proxy.
- At 50 Hz: 458 left-edge, 542 right-edge, one both-side and one unloaded sample;
  180 direct left/right support changes. A single unloaded sample does not prove
  flight; the substep flight diagnostic finds no airborne interval.
- Within-cycle body yaw has 5.93° mean peak-to-peak excursion (6.38° maximum).

The much larger head yaw span is not the body tilt or body yaw span. These values
show why increasing head amplitude alone is not proof of reproducing the desired
large body-rocking style. All values are uncalibrated simulation diagnostics.
The report is `experiments/reports/motor-training-gait-styles-baseline-2026-11-19.json`.

## Separate useful oscillation from drifting off course

Measure these independently:

- **Within-cycle torso yaw:** the intended left/right rotation during a step.
- **Cycle mean and same-phase heading:** rotation that persists from one full
  left/right pair to the next.
- **Travel direction and lateral displacement:** the path relative to the fixed
  requested forward axis; body orientation and velocity direction can differ.

The preserved rapid optimizer constrained maximum instantaneous heading error to
20°. The large-rocking optimizer now checks heading at every full-cycle endpoint.
Its experimental controller uses delayed same-phase heading feedback; bounded
yaw-bias correction improves short trials but has not solved longer-run drift.

**Confirmed by the user:** the 20° tolerance applies after each complete
left/right cycle; larger temporary torso excursions are allowed. Measure every
cycle endpoint against the original startup heading, including startup cycles;
do not reset the reference each cycle or check only the final endpoint. Keep
within-step extrema visible. The 35° simulation fall guard is unchanged.

## Training and acceptance changes

Preserve `configs/sway-faster-trained.json` and its feedback/motor configuration
as the rapid style. Give the large-step family separate configs and reports.
Search motion frequency, roll/yaw phase, stroke/hold/braking timing, and reachable
pose amplitudes under the existing physical and software motor limits.

For the large-step family, measure signed forward displacement over a full
left/right pair, progress associated with each support transfer, opposing-rim
lift, torso roll/yaw, and reliable landing/return. Do not reward rocking in place,
count positive half-cycles while ignoring backward returns, or maximize tilt
without a bounded stable envelope. Treat a larger step as a separate style
criterion so a rapid small-step policy cannot win merely by increasing frequency.
Choose numeric style thresholds from the existing baseline and calibrated
demonstrations; none is presented as an established hardware limit here.

Contact forces and contact locations are useful simulator diagnostics. A future
physical policy must use signals actually available on the robot or a calibrated
external estimator; do not silently depend on simulator-only contact observations.
The head remains the driver; torso turning is an outcome to measure, not an
unreported added body-yaw command.

Evaluate each style over sustained runs, multiple physics timesteps and held-out
model variations. Preserve all falls and failed starts. Mode transitions need
their own balance, actuator and heading checks before exposing a style selector.

## Source review

The [Vimeo review](VIMEO_OBSERVATIONS_2026-11-18.md) records earlier actual playback.
On this follow-up, both browser players returned "Unable to play media".
The [saved GIF](GIF_OBSERVATIONS_2026-10-28.md) was re-inspected at frames 0–20 and
42–62: head turning/tilting accompanies torso rocking, with apparent progress
later in the clip. Its separate fall/reset evidence remains in the original
annotation. No fresh Vimeo playback or metric physical measurement is claimed.
