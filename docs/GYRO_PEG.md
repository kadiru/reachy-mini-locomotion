# Rolling Reachy Mini around its base

**CAD-foot follow-up.** The deep controller reproduced its rim roll on the circular contact proxy. Switching floor contact to the CAD shells caused the same controller to finish flat. Treat the deep roll as a geometry-sensitive simulation result. [Matched experiment and tilted-release films](TILTED_RELEASE_AND_SIDE_ROLL.md)

The target is a tilted, free-standing base whose loaded point travels around the circumference of the foot. Two different circles can appear in the same run. On a short timescale the contact patch travels around the 112 mm foot. Over many wobbles the center of the whole base can trace a much wider circle on the floor.

## Sustained deep roll

The side-balance regulator can now hand control around the entire foot. Its selected blend starts on the left edge and makes **2.33 continuous rim revolutions** with a **31.17° median lean** after capture. In this 36.7-second nominal run, the contact traced **13.1 cm diameter** circles on the floor, while the center of the foot traced **3.5 cm** circles. The waist target stayed at zero. [Watch the deep roll](../media/simulation/gyro-peg-deep.mp4)

Extending the same nominal run to **65 seconds** produced **4.35 uninterrupted revolutions** around the foot. The regulator acquired the initial left edge at 3.16 s. From capture onward, every interval in the 8 kHz audit had loaded rim contact. Median lean was **31.04°**, peak lean was **32.70°**, and base yaw changed by **−233.7°**. The loaded material's floor-relative slip distance was 0.112% of the arc swept along the rim. The episode finished upright with joint clearance.

The initial rotating side regulator lost balance after at most 0.84 turns in the longer trials. The blend expresses the independently fitted front/back and left/right regulators in the moving lean frame and interpolates between adjacent sectors. Five of twelve training runs met the two-turn, 25° median-lean criterion, including both launch sides. The selected 14-second reference period smooths the transition out of the initial static hold. The film shows the natural launch and both full turns in real time, followed by a fixed overhead replay.

The selected controller passed **0/4 reserved deep-roll trials**. All four failed to establish the initial edge balance. On the lower-grip model and on the lower-voltage and reduced-gain model, Reachy settled back onto the flat foot. With 0.03° attitude noise it reached a joint stop at 3.94 s. With a finer 0.10 ms physics step it reached a joint stop at 4.55 s. The nominal recorded trial finished with 0.048 rad of joint clearance and a net base yaw of −128.4°. [Frozen deep controller](../configs/gyro-peg-deep-experimental.json)

## Wide circles in two-minute trials

The best reserved run traced a **20.43 cm diameter** circle on the higher-grip floor, with 354.4° of sweep in 61.8 seconds. The base center finished the lap **9.5 mm** from where it started. Its median lean over the episode was **12.74°**, peaking at **24.52°**. The short ground-contact loops measured about **11.36 cm** across during this long traveling lap. Each physical sector of the foot takes a turn carrying load.

On the nominal floor the positive-mirror run completed two disjoint wide laps, approximately 14.22 and 13.89 cm in diameter. Its mirrored counterpart completed three. The physical material-point speed at the loaded foot was small relative to the distance the contact progressed along the circumference. In the filmed high-grip trial the 50 Hz slip estimate was 0.87% of that rim arc. [Watch the measured circle](../media/simulation/gyro-peg-wide.mp4)

| Reserved condition | Positive mirror | Negative mirror |
| --- | --- | --- |
| Finer timestep, 0.25 ms | 14.19 cm, pass | 11.55 cm, pass |
| Higher grip, friction 0.827 | 20.43 cm, pass | 16.29 cm, pass |
| Slick, friction 0.567 | 10.67 cm, below the diameter target | 9.51 cm, below target |
| Redistributed mass and two-frame command delay | Fell | Fell |

Four of eight reserved trials passed the full wide-circle test. Six of eight completed the two minutes and passed the underlying rim-roll test. The two falls occurred on the same changed mass and motor-delay model. Both nominal trials passed. An earlier search candidate reached a 28.8 cm fitted circle at only 8.8° median lean, below the 12° requirement. [Frozen wide-circle controller](../configs/gyro-peg-wide-experimental.json)

## What changed from the earlier spin

Replaying the original requested-heading policy on its four-stroke slick-floor example reproduced **88.32° of torso turn**. Under the new measurements the base leaned **3.33° at the median**, peaked at **24.78°**, and loaded the rim across **43.6%** of intervals. The contact already wound as far as **2.27 uninterrupted revolutions** during that pivot. The new policy sustains that part of the movement.

On the featured wide-circle run, an independent contact audit at **2,500 physics steps per second** found **98.13% loaded intervals**, a longest continuous contact run of **7.82 rim revolutions**, and **0.85% material slip per rim arc**. The two nominal 24-second rim-roll audits at 2 kHz recorded continuous runs of 13.70 and 40.38 rim revolutions. All waist target samples were zero.

The 24-second rim-roll policy passed **7/8 reserved trials**. Its nominal median lean was 13.1° in one direction and 13.4° in the other. The earlier rim-spin policy, replayed under the same nominal conditions, had median leans of 5.9° and 2.7°. Neither earlier rim-spin trial completed an uninterrupted material-contact revolution under the new test. Switching off phase feedback on the new head waveform cut nominal median lean to 6.7° and 7.2° and broke continuous contact before a full revolution. The earlier spin controllers still meet their own rotation objectives.

## What the controller does

For the wide circle, the six head actuators command a coupled roll, pitch, yaw and lateral orbit at 50 Hz. A bounded phase correction uses delayed simulated base roll and pitch to speed up or slow down the head motion as the foot leans. The policy can turn its phase reference over for a mirrored run. Sensing uses the delay, attitude noise and seed specified for each model.

For the deep roll, the controller also reads the simulated head pose. It waits for 25 consecutive 50 Hz frames with the sensed roll within 2° of the target, sensed roll rate below 5°/s and current simulated pitch within 5°. It then advances the direction of lean around the base at one reference revolution per 14 seconds after startup. The four fitted balance regulators supply the head corrections as the reference crosses from one edge to the next.

Both controllers start from rest on a flat base and keep the waist servo at a constant zero-radian target. All motion comes through the existing battery torque–speed approximation, PWM limits, command profile and checked head kinematics. Only the evaluator reads contact force, floor coordinates and world heading.

The first search tested 162 periodic head motions and refined 100 before testing 24 finalists. The tilt-feedback search then evaluated 144 timings, refined 96, and checked 28 finalists on two training surfaces in both directions. For the wide floor circle we varied asymmetric head posture and yaw around these motions, screened 48 new runs and ran five finalists for two minutes each. Policies were frozen before the reserved runs. A separate deep-lean experiment captured the existing 32° balance posture and rotated the regulator's reference around the foot. These are searches over feedback controllers in CPU MuJoCo.

## How a circle is counted

- **On the foot.** Local contact is the normal-force-weighted center of pressure transformed into the moving foot frame. Samples count when the base leans at least 5°, the foot carries more than 0.5 N, and contact lies at least 85% of the foot radius from its center. We only unwrap adjacent valid samples, and reject angular jumps of 90° or more. A loss of contact interrupts the lap. A separate audit repeats these measurements at every physics integration step.
- **Low-slip rim motion.** We measure the floor-relative velocity of the foot material at the current center of pressure. Its integrated horizontal travel divided by the material arc swept along the rim measures slip. The gyro-peg test requires a complete run with joint clearance, at least three uninterrupted rim revolutions, at least 80% loaded intervals, at least 90% of angular travel in the commanded direction, median lean of at least 12°, and slip below 25% of the rim arc. After the 1.5-second launch allowance, unsuccessful and interrupted intervals stay in the denominator.
- **On the floor.** A single-wobble floor circle and a wide traveling circle use separate measurements. To measure the latter, we smooth the *foot center* positions across 1.02 seconds and sample at 5 Hz. The circle fit must sweep 350–385°, make at least 93% of its angular travel in one direction, keep the standard deviation of radius below 18% of the mean, and finish within 35% of one radius of its start. A wide pass requires a diameter of at least one full foot width (112 mm), the full gyro-peg criteria, and sustained loaded contact and lean during the fitted loop. Repeated laps use disjoint time windows. These are experimental thresholds chosen for this behavior.
- **Falls.** Body contact with the floor and joint-limit stops are recorded separately. Physics has no tilt-angle cutoff for these runs. A guard-stopped trajectory counts as a failed trial.

## Reproduce

Six training searches used **13.62 Wh** gross system energy over **645 rollouts** on the Apple M4 CPU. At an illustrative $0.30/kWh that is **$0.00409**. The rim-roll search used 5.91 Wh. Extending it to wide circles brought its cumulative energy to 8.59 Wh. Developing the deep roll from the existing edge-balance regulators added 5.03 Wh. These measurements cover the new fitting runs and exclude earlier balance training, evaluation and rendering. Training used zero GPU-hours. [Raw receipts and per-model compute](../experiments/reports/training-compute.csv)

To reproduce the films from the frozen controllers, run the following commands from the project root with the pinned MuJoCo assets. The replays check their motion metrics against the archived deep nominal trial and wide held-out trial.

```sh
.venv/bin/python experiments/refine_traveling_edge.py replay
.venv/bin/python experiments/film_gyro_peg.py experiments/generated/gyro-peg/sector-best.json --report experiments/reports/motor-training-gyro-peg.json --output experiments/generated/gyro-peg/deep-replay.mp4
.venv/bin/python experiments/fit_gyro_course.py replay
.venv/bin/python experiments/film_gyro_peg.py experiments/generated/gyro-peg/wide-best.json --report experiments/reports/motor-training-gyro-peg.json --output experiments/generated/gyro-peg/featured-replay.mp4
```

To repeat training, use the project environment. The power meter wraps each training command with a separate 15-second background interval. Inspect the receipts for the exact commands. Qualification and video rendering are separate.

```sh
.venv/bin/python experiments/train_gyro_peg.py fit --workers 8
.venv/bin/python experiments/fit_gyro_feedback.py --workers 8
.venv/bin/python experiments/fit_traveling_edge.py fit --workers 6
.venv/bin/python experiments/fit_gyro_course.py fit --workers 8
.venv/bin/python experiments/fit_gyro_course.py qualify --workers 8
.venv/bin/python experiments/train_gyro_peg.py qualify --workers 8
.venv/bin/python experiments/audit_gyro_peg.py --output experiments/generated/gyro-peg/audit.json
.venv/bin/python experiments/fit_traveling_edge.py endurance --workers 6
.venv/bin/python experiments/fit_traveling_edge.py qualify --workers 6
.venv/bin/python experiments/refine_traveling_edge.py fit --workers 6
.venv/bin/python experiments/refine_traveling_edge.py qualify --workers 6
.venv/bin/python experiments/refine_traveling_edge.py audit
```

With the six matching power receipts in the work directory, run `.venv/bin/python experiments/archive_gyro_peg.py` to rebuild the [experiment report](../experiments/reports/motor-training-gyro-peg.json). It includes the frozen policy parameters, individual training and evaluation outcomes, and contact audits.

The 56 mm-radius circular contact proxy, motor-bus voltage, traction, IMU mounting and estimator response remain uncalibrated on the physical robot. [Battery motor model](MOTOR_LIMITS.md) · [Earlier head-driven spin](CONTINUOUS_SPIN.md) · [Original heading-conditioned edge pivot](SLICK_QUARTER_TURN.md)
