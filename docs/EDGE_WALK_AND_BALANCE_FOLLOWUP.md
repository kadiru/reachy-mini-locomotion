# A forward step on alternating rims

Starting flat, Reachy can now advance at one directional head swing per second while slowing its head yaw on alternating loaded rims. In the filmed 24-swing trial it advanced 25.65 mm, lifted one side by 11.61 mm and finished settled on its base. Measured braking coincided with strict edge support on 10 left-rim strokes and 11 right-rim strokes. Its peak body tilt was 5.95°. This is a small edge step. A near-toppling forward leap remains an open target. [Full real-time walk](../media/simulation/alternating-edge-brakes.mp4)

| Frozen policy | Forward in 24 active seconds, starts −1 / +1 | Measured edge-braking strokes, filmed +1 start | Largest absolute heading at a pair end, +1 start |
|---|---:|---|---:|
| Earlier shallow walk | 70.97 / 70.90 mm | 0 | Under 3° |
| Faster edge walk | 34.41 / 37.61 mm | 11 left, 1 right | 18.38° |
| Repeated alternating edge walk | 25.63 / 25.65 mm | 10 left, 11 right | 14.49° |
| Head-encoder braking | 11.42 / 11.45 mm | 10 left, 6 right | 7.24° |

The higher-speed candidate favours one rim. The earlier shallow walk remains the distance leader at the same stroke count. All four use the zero waist target and the bounded battery servo model. The three new policies also use a delayed, noisy heading estimate sampled at the start of each full pair. No angle cutoff terminates their episodes. [Faster edge walk](../media/simulation/fast-edge-cadence.mp4) · [Earlier shallow walk](../media/simulation/teleop-cadence.mp4)

## What moved the robot

The search first varied head roll, yaw, lateral motion and stroke timing. Later searches added two feedback mechanisms. Pair-end heading feedback biases the next pair's yaw timing or target. A separate controller uses a 20 ms delayed roll, roll-rate and relative head-yaw measurement. It can interrupt the head command while the simulated head is still moving toward its commanded endpoint. Floor contacts enter the evaluation only. This is parameter search in MuJoCo. The operator's Quest demonstration supplied the cadence and motion hypothesis. Training has not used a time-synchronized recording from the physical robot.

In the filmed alternating gait the heading controller was enabled. The within-stroke roll triggers never fired. Braking on the rim came from timing the head command and allowing the bounded servo to slow the head. The encoder-braked alternative interrupted the command on 23 of 24 strokes. These are different mechanisms. The measured results establish when the base moved and pivoted. They do not isolate how much momentum transferred from the head versus impulse from the ground.

A strict edge requires at least 10 mm between rim heights, over 0.5 N on the loaded side and under 0.1 N on the opposite side. A measured braking sample requires actual head yaw speed above 20°/s and falling by over 100°/s². In the filmed alternating trial, 2.15 mm of signed forward movement and 2.78° of cumulative absolute base yaw occurred in intervals with strict support at both ends. Its largest change between consecutive pair-end headings was 2.60°. The 14.49° figure measures accumulated heading relative to the starting direction. Both heading measures allow larger within-step torso swings.

An earlier unsteered attempt braked on both edges and gained 26.27 mm but reached 23.13° from its starting direction. The film includes the complete run and settle. [Heading failure](../media/simulation/edge-walk-heading-miss.mp4)

## Forty matched walking tests

Each of five previously unused combinations of battery voltage, friction, delay, gain and mass distribution was tested from both starts for 16 active seconds per policy. All 40 finished without falling. The faster edge walk passed 10/10, advanced 11.1–30.4 mm, and braked on both rims in all ten. The shallow comparison passed 10/10 and advanced 50.3–76.9 mm. It braked on both rims in two conditions. The alternating walk braked seven times on each rim in every trial. It passed 9/10. It advanced 20.63 mm in the failed trial at friction 0.890 and 5.44 V, but the pair-end heading reached 27.19° from the start. The encoder-braked walk passed 7/10. With its within-stroke sensor triggers muted, the nominal encoder policy travelled farther on both starts. For these trials, the additional feedback did not buy more distance per swing.

The three new walks passed from both starts when the nominal physics timestep was halved to 0.125 ms. The chaptered film shows all 40 matched cases in ten chapters. Each chapter puts four policies on the same condition and labels the final distance, signed starting side, pair-end heading and left/right braking counts. [Watch all 40 tests](../media/simulation/matched-edge-walks.mp4) · [Three-policy comparison](../experiments/reports/motor-training-matched-edge-walks-2026-11-21.json) · [Alternating gait on the same conditions](../experiments/reports/motor-training-alternating-edge-walks-2026-11-21.json) · [All 575 search and verification runs](../experiments/reports/motor-training-edge-walk-search-2026-11-21.json)

The earlier walk was the fastest of the four at one swing per second. On floor 2 it advanced 76.92 mm in 16 swings, reached 18.66 mm peak rim lift and braked seven times on each rim. Its maximum pair-end heading was 6.60°. It produced repeated braking on both rims on this particular floor; the dedicated alternating gait did so on all five. [Fastest matched walk](../media/simulation/earlier-walk-fastest-matched.mp4)

## Retraining on the 27.19° heading miss

The repair search fit the amount of yaw correction permitted per step and the difference in yaw timing between the two directions. Seven of 15 candidates passed the exact previously failed trial. The selected gait combined heading-dependent timing with a larger allowable yaw correction. On that same floor and start it advanced **28.84 mm in 16 strokes**, with a **26.00 mm rim lift** and **7.10° maximum pair-end heading**. Peak body tilt was 13.61°. It slowed the head on seven left and seven right rims. Over strict same-rim contact intervals the base accumulated 32.02° of absolute twist and advanced 19.16 mm. The head followed timed strokes with pair-end heading feedback throughout this run. This case was used in fitting. [Watch the repaired trial](../media/simulation/edge-heading-repair.mp4) · [Policy](../configs/cadence-alternating-repair-experimental.json) · [Fit, repeated conditions and fresh tests](../experiments/reports/motor-training-cadence-heading-repair-2026-11-21.json)

The repair passed all ten original tests, all ten new floor/motor/start combinations and both 40-second nominal runs. The fresh 16-second trials advanced 13.77–39.60 mm with no falls, and their worst pair-end heading was 11.56°. In the fastest fresh trial, the rim height difference reached 29.33 mm at 15.44° peak tilt. Seven strokes braked on each rim. The base advanced 31.57 mm in strict same-rim intervals and 39.60 mm overall. Total absolute twist on a loaded rim was 44.30° and maximum pair-end heading was 7.00°. The two 40-swing nominal runs reached 44.48 and 46.62 mm with a maximum pair-end heading of 16.79°. Nominal 24-swing distance dropped from 25.63–25.65 mm to 23.06–23.07 mm after repair. The physical sensor pipeline and contact model remain uncalibrated. [Fastest fresh trial](../media/simulation/edge-walk-fastest-fresh.mp4) · [All 22 post-repair checks](../media/simulation/edge-heading-repair-all.mp4)

## Balance tasks that remain open

- **Front and back from flat.** A new search varied launch frequency, phase, capture threshold, prediction and transition time on both edges. None of 32 training and four finer-step confirmation trials held the edge for the final five seconds. Twelve training episodes made non-foot contact with the floor. Two filmed runs show the lower torso striking at 72.58° and 70.59° body tilt. The 0.125 ms confirmation launches peaked at 27.64° front and 20.59° back. The prepared-start holds near ±32° remain a separate result. [Both filmed landings](../media/simulation/foreaft-launch-landings.mp4) · [All 36 conditions](../experiments/reports/motor-training-foreaft-natural-2026-11-21.json)
- **Balance using one gyro sample every 20 ms.** Seven blends of instantaneous gyro rate with the rate derived from successive simulated attitude measurements were fitted on both edges. The selected blend held 0/6 fresh-seed trials. The original single-sample controller also held 0/6. Training-time high-rate gyro averaging held 2/6 on the same seeds. In one failed blend trial the body hit the floor. Five reached head joint travel. Hardware IMU filtering, timing, attitude quality and head-pose accuracy remain unmeasured. [Training and matched controls](../experiments/reports/motor-training-sampled-balance-2026-11-21.json)
- **Direct side hits.** The trial activated a head tracking-error limit after a stable edge hold and a sensed disturbance. Four nonzero settings and the original controller were trained/evaluated on both incoming sides of both edges with 10 g balls released at 1.4 m/s. None repaired these four training failures. On 12 fresh conditions the original recovered 5/12, and the selected nonzero challenger recovered 2/12. All 24 comparison impacts were direct and had qualified pre-hit balance. The original controller remains the better choice here. [Direct-hit training and comparison](../experiments/reports/motor-training-direct-recovery-2026-11-21.json) · [Earlier successful direct impacts](../media/simulation/direct-airborne-recoveries.mp4)

## Reproduce

Use the project environment, the pinned upstream model and fresh output paths. All commands below run locally on CPU. The search and qualification reports retain unsuccessful runs.

```sh
root=experiments/generated/edge-walk-reproduction
.venv/bin/python experiments/train_edge_timed_walk.py --out "$root/initial"
.venv/bin/python experiments/train_edge_timed_walk.py --refine "$root/initial" --out "$root/repeated"
.venv/bin/python experiments/train_edge_timed_walk.py --refine "$root/initial" --encoder-catch --out "$root/encoder"
.venv/bin/python experiments/fit_pair_heading.py --source "$root/repeated" --out "$root/heading-timed"
.venv/bin/python experiments/fit_pair_heading.py --source "$root/encoder" --out "$root/heading-encoder"
.venv/bin/python experiments/compare_edge_walks.py --timed "$root/heading-timed" --encoder "$root/heading-encoder" --states "$root/matched-states" --out "$root/matched.json"
.venv/bin/python experiments/compare_edge_walks.py --alternating-only --timed "$root/heading-timed" --encoder "$root/heading-encoder" --states "$root/alternating-states" --out "$root/alternating.json"
.venv/bin/python experiments/film_matched_walks.py --report "$root/matched.json" --states "$root/matched-states" --alternating-report "$root/alternating.json" --alternating-states "$root/alternating-states" --work "$root/chapters" --output "$root/comparison.mp4"
.venv/bin/python experiments/train_cadence.py --replay configs/cadence-alternating-experimental.json --side 1 --seconds 24 --out "$root/nominal"
.venv/bin/python experiments/render_sway.py "$root/nominal/walk.json" --output "$root/nominal.mp4" --slowdown 1 --diagnostics --frame-path --follow-front
.venv/bin/python experiments/fit_foreaft_launch.py --out "$root/foreaft.json"
.venv/bin/python experiments/fit_sampled_balance.py --out "$root/sensors.json"
.venv/bin/python experiments/fit_direct_recovery.py --out "$root/direct.json"
.venv/bin/python experiments/repair_cadence_heading.py --out "$root/repair.json"
.venv/bin/python experiments/film_heading_repair.py --work "$root/repair-film" --output "$root/repair-film.mp4"
```
