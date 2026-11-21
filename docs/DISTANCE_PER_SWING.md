# How far does one head swing move Reachy Mini?

The selected head motion moves the simulated base **1,221.86 mm in 160 swings** from an upright start to a settled stop. That is **7.64 mm per swing** across 80 left–right pairs. The simulated floor, battery and motor models match the comparison runs. The video plays the complete 25.6 seconds at real speed and displays the running count. [Watch the run](../media/simulation/distance-per-swing.mp4).

## What the policy learned

A lateral head shift with a 24 mm amplitude accompanies about 27° of commanded head roll and 28° of yaw. The yaw stroke is offset from roll by 122°. The 50 Hz controller rounds the cycle to 15 samples, giving 3.33 left–right pairs per second. Yaw moves for 77% of each half-cycle. The brake occupies about 4% of that commanded stroke. The bounded servo determines how closely the head follows this command. Heading feedback uses the simulated, delayed external base-heading estimate. The waist holds a zero target throughout.

In the filmed trial the rim height difference reaches 19.74 mm. Within a pair, torso rotation averages 10.23° peak to peak and reaches 12.57°. Floor-relative **base** rotation averages 8.96° peak to peak. The largest heading error measured after a complete pair is 13.59°, relative to the original start. This supports the idea that larger intrastep turns can accompany longer steps. The experiment varied multiple motion parameters together, so it cannot isolate braking as the cause.

## Counting movements

Every trial includes amplitude ramp-up, head return and 1.6 seconds after the last pair for the return and settling. Forward distance is the displacement of the base along its initial forward axis. Sideways drift and a curved path cannot add to the score.

One swing is one directional half of a compound head motion. We charge the greatest of three counts over the full trial: commanded half-cycles, actual head-roll strokes and actual head-yaw strokes. An actual stroke starts when motion changes direction and travels at least 10° from the preceding extreme. A roll and yaw performed together count as one compound movement. Extra oscillations during braking or settling can increase the count. Tiny corrections and total angular travel are different cost measures. The filmed result also counts as 160 swings at the 6° and 15° thresholds.

## Matched nominal comparisons

All values below use 0.25 ms physics, the same starting side as the video, a 5 V battery-bus assumption, the same head torque/speed envelope and the same neutral return. Every scored pass completes the requested budget, stays within ±20° after each pair and at final rest, and settles upright. No 35° angle cutoff is applied. Physical floor contact, infeasible inverse kinematics, actual joint limits, closure errors and numerical failures terminate the run.

| Controller | Pair budget | Forward distance | Counted swings | mm per swing | Outcome |
|---|---:|---:|---:|---:|---|
| Earlier rapid headshake | 24 | 281.13 mm | 49 | 5.74 | Pass |
| Earlier rocking gait | 24 | 318.74 mm | 48 | 6.64 | Pass |
| New distance-trained gait | 24 | 332.50 mm | 48 | 6.93 | Pass |
| Earlier rapid headshake | 80 | 985.04 mm | 161 | 6.12 | Pass |
| Earlier rocking gait | 80 | 1,023.26 mm before its guard stop | 160 budgeted | — | Infeasible head target |
| New distance-trained gait | 80 | 1,221.86 mm | 160 | 7.64 | Pass |

The sustained edge-pivot walker is a distinct, deeper-rocking style. Its archived 20-pair runs produce 633–651 mm with 396–401 measured head-roll strokes. That gives **1.58–1.64 mm per swing** when evaluated by the same counter. Internal pumping and repeated attempts account for the difference between primitive bursts and individual head strokes. This archive comparison uses the sustained controller's own run length.

## How many swings does it take to go 10 cm and stop?

We ran every whole-pair budget from 1 through 14 for each gait and starting side, for 84 independent starts. The table gives the smallest counted swing total among runs that crossed 100 mm, satisfied the heading limit and settled flat.

| Gait | Starting side +1 | Mirrored start −1 |
|---|---:|---:|
| New distance-trained gait | **18 swings**, 100.55 mm | **20 swings**, 113.48 mm |
| Earlier rocking gait | 20 swings, 104.91 mm | 22 swings, 110.59 mm |
| Earlier rapid headshake | 21 swings, 105.75 mm | 23 swings, 107.07 mm |

## Where it still fails

Both starting sides pass 24 nominal pairs. The filmed 80-pair starting side also passes at 0.125 ms with 1,224.31 mm, 160 swings and 13.45° maximum pair-end heading error. On the mirrored 80-pair test, inverse kinematics rejects an unreachable target after 61 pairs at 0.25 ms and after 74 pairs at 0.125 ms.

The frozen policy passes **8/12** new 24-pair stress runs. Three walks finish upright outside the heading limit. One mirrored run reaches an infeasible target after 19 pairs. Its subsequent outcome is unknown. The next training target is steering with the head joint ranges explicitly considered when requesting a correction. The simulated 5 V motor supply and the floor contact and sensor models still need hardware calibration.

## Training and reproduction

The first seeded evolutionary parameter search generated 96 new head motions. A second search generated 180 further candidates and used longer selection runs. Mutations varied roll, lateral shift, yaw amplitude, phase, frequency and braking. Selection rewarded signed millimetres per swing among completed walks. A final search compared 27 new steering settings plus the incumbent on 56-pair walks from both starting sides. Five finalists ran 24- and 80-pair tests at 0.25 ms before the final policy was frozen. Fresh conditions vary friction, motor gain and latency, bus voltage, mass distribution and heading sensing.

Paths below are relative to the project root. Use fresh output directories.

```sh
.venv/bin/python experiments/train_swing_distance.py --output experiments/generated/stride-initial
.venv/bin/python experiments/train_swing_distance.py --refine-from experiments/generated/stride-initial/policy.json --output experiments/generated/stride-refined
.venv/bin/python experiments/refine_swing_steering.py --policy experiments/generated/stride-refined/policy.json --output experiments/generated/stride-steering
.venv/bin/python experiments/train_swing_distance.py --replay configs/head-swing-distance-trained.json --pairs 80 --side 1 --dt .00025 --output experiments/generated/stride-replay
.venv/bin/python experiments/render_sway.py experiments/generated/stride-replay/walk.json --output experiments/generated/stride-replay.mp4 --slowdown 1 --diagnostics --frame-path --follow-front --title 'DISTANCE PER HEAD SWING'
.venv/bin/python experiments/measure_swing_budget.py --policy configs/head-swing-distance-trained.json --output experiments/generated/stride-target.json
```

[Saved policy](../configs/head-swing-distance-trained.json) · [Training, qualification and replay measurements](../experiments/reports/motor-training-swing-distance-2026-11-20.json) · [Every 10 cm budget test](../experiments/reports/motor-training-swing-distance-goal-2026-11-20.json).
