# More torso rotation on a loaded rim

Reachy Mini starts upright with its base free. The waist servo holds zero. The
controller uses the same battery motor envelope, actuator delays, 50 Hz command
rate, joint-travel guard and floor-contact checks as the earlier target-pivot
policy. There is no angle cutoff. All measurements are from MuJoCo.

## Results

The frozen policy passed **92/95** simulated turns, including
**67/67** on the original suite and **15/18** on three
reserved combinations of surface, battery model, mass distribution, command
delay, sensor noise and integration step. The other ten trials add quarter-turns
to previously tested conditions. **20/20** previously failing
trials now pass. These include all six former right-turn joint stops and all 14 former
low-grip failures. Individual outcomes are in the [qualification report](../experiments/reports/motor-training-rim-efficiency.json).

| Condition | Request | New torso heading | Strokes, old → new | Measured swings, old → new | Torso degrees on rim per swing, old → new | New max center shift |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Nominal | -90° | -90.1° | 9 → 4 | 51 → 34 | 1.40 → 2.42 | 48.6 mm |
| Nominal | +90° | +86.3° | 9 → 5 | 53 → 39 | 1.37 → 1.90 | 39.8 mm |
| Slick floor, friction 0.55 | -90° | -89.9° | 9 → 8 | 91 → 42 | 0.22 → 1.71 | 36.5 mm |
| Slick floor, friction 0.55 | +90° | +86.9° | 9 → 5 | 99 → 64 | 0.32 → 0.98 | 48.4 mm |
| Uniform mass, 40 ms command delay | -90° | -89.5° | 2 → 5 | 5 → 30 | 2.84 → 2.60 | 49.5 mm |
| Uniform mass, 40 ms command delay | +90° | +88.6° | 7 → 4 | 41 → 26 | 1.90 → 2.84 | 46.7 mm |

The delayed uniform-mass baseline stopped during its second rightward stroke.
Its measurements describe only the motion before the guard. Both quarter-turns on the
friction-0.60 floor still needed all nine strokes. These results use the
uncalibrated battery and contact model.

Three reserved cases missed the qualification criteria.

- validation slick, target -90°, settled torso -91.1°, base yaw on the rim -42.0°, peak center excursion 64.6 mm.
- validation slick, target +90°, settled torso +74.5°, base yaw on the rim +35.1°, peak center excursion 46.1 mm.
- validation middle, target -24°, settled torso -22.9°, base yaw on the rim -8.2°, peak center excursion 19.3 mm.

Recorded body-floor contacts totaled 0. Joint-travel guard stops totaled 0.

[Watch the best qualifying turns](../media/simulation/efficient-rim-turns.mp4) · [Frozen policy](../configs/efficient-rim-pivot.json)

## What counts as progress

A stroke is one launch, rock, and return to neutral. It can contain several
directional head swings. We count actual head roll and yaw separately using
10° reversal hysteresis. The swing count is the larger of the two axis counts. This avoids double
counting synchronized roll and yaw and can undercount motions on separate axes. Comparisons use the same counter on
both policies. This is a repeatable proxy for hand/controller movements.

Signed torso rotation accumulates only between consecutive 20 ms samples on
the same loaded side. Reverse rotation subtracts. Each sample requires at
least 10 mm of rim height difference and more than 0.5 N on the loaded side,
with less than 0.1 N on the opposite side. Base yaw is measured independently.
The waist servo can deflect under load, making the separate measurements
important.

A qualifying turn must complete and settle, with both torso and base within 5°
of the command throughout the final 0.6 s. Both must accumulate at least 40%
of the requested turn on loaded rims. Maximum center excursion is 56 mm and
mean integrated rim slip is limited to 15 mm per stroke. The stroke budget is
nine. For the video, qualifying 90° turns with nonnegative joint clearance are ranked by stroke count, measured
head swings, then signed torso rotation on the rim per measured swing. The
film also includes the highest-ranked slick-floor run and delayed right turn
when those differ from the overall winner.

## How the policy is fitted

The empirical policy search used 2,164 single-stroke episodes, followed by
648 continuous two-stroke episodes for 108 signed actions on six conditions.
Searches vary the preload, rocking frequency, roll/rate catch, yaw timing,
braking and return. Additional searches try winding the head up before launch,
a fore-aft head stroke during the turn, and yaw oscillation synchronized with
roll. Seeded evolutionary and local searches rank settled motions by signed
edge rotation, heading gain, actual swing count, and center travel. The final
fit evaluates mirrored launches twice in one continuous simulation on six
conditions. The repeated launch exposes joint excursions that a single
upright start can miss.

The policy consults the measured first-stroke and repeated-stroke responses.
Candidates that crossed a guard on a plausible model are excluded for that
phase. For requests of 60° or more, full quarter-turn trials also select an
opening by requested direction and known observation delay. Connected training turns also record unsafe action transitions, keyed by
the preceding motion and its observed angular response. After landing, observed heading change and peak roll update the
relative weights of the response tables. Training also selects the opening
for short 14–20° requests and tunes model confidence and center-travel cost
for large turns with two-frame sensing delay. A further candidate learns a
short sequence from the response to a diagnostic stroke. It selects a
sequence only when the first observed landing distinguishes the learned
responses. For a remaining miss, the trainer replays alternative final
strokes from the same observed approach. The final selection compares these
candidates on complete turns. The next stroke is scored for
remaining heading error, edge turn, head movements and predicted center
travel. Floor friction, contact forces and ground-truth center position are
used during training and evaluation. The deployed simulation policy selects
actions from delayed, noisy roll and relative-heading observations and their
configured latency. The physical sensor pipeline still needs calibration.

## Reproduce

Run in the project environment with the pinned upstream robot assets.

```sh
for stage in baseline search windup surge phase evolve local robust fit openings transitions small-turns slick-feedback probe-plans finish-repair freeze paired qualify
do
  .venv/bin/python experiments/train_rim_efficiency.py "$stage" --workers 6
done
.venv/bin/python experiments/train_rim_efficiency.py best
.venv/bin/python experiments/film_target_pivot.py experiments/generated/rim-efficiency/featured-*.json --slow-edge --output experiments/generated/efficient-rim-turns.mp4
.venv/bin/python experiments/train_rim_efficiency.py replay --condition nominal --goal 90 --out experiments/generated/efficient-left.json
.venv/bin/python experiments/film_target_pivot.py experiments/generated/efficient-left.json --output experiments/generated/efficient-left.mp4
```
