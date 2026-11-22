# A 31° rolling circle and a hanging-ball test

**CAD-foot follow-up.** The deep controller reproduced its rim roll on the circular contact proxy. Switching floor contact to the CAD shells caused the same controller to finish flat. Treat the deep roll as a geometry-sensitive simulation result. [Matched experiment and tilted-release films](TILTED_RELEASE_AND_SIDE_ROLL.md)

The experiment starts the wireless Reachy Mini from its flat foot in MuJoCo. The waist servo holds zero throughout. The head uses the original battery torque–speed model, with torque capped at 80% of the modeled stall value and an 80% PWM limit. The command profile caps target speed at 10.7 rad/s and acceleration at 360 rad/s² and includes one frame of motor delay. Inverse kinematics and actual joint travel are also checked. The nominal bus voltage is 5.00 V. Floor traction, loaded voltage and mass properties still need physical calibration. [Motor model and source data](MOTOR_LIMITS.md)

The widest qualifying endurance lap was **3.59 cm in diameter** on the 4.95 V model. Reachy captured the edge at 3.40 s and stayed upright for 75 s. From 15.90 to 37.70 s it swept 361.5° and returned within **0.46 mm** of the lap start. Median lean during the lap was **30.84°** with a 30.21–33.03° range. The unsmoothed center path measured 3.61 cm across. The loaded contact traced about **13.1 cm** circles and moved through 3.92 uninterrupted revolutions around the foot over the trial. Net angular motion about the floor normal during the fitted lap was only 0.14°. The earlier deep roll measured about 3.5 cm across at the foot center. [Watch the measured lap](../media/simulation/deep-lift-center-circle.mp4)

## Measuring the motion

The controller observes the head pose and delayed roll and pitch during the first 1.18 seconds. It compares the measured lift with the training examples before choosing a braking and capture correction. The noisy-sensor candidate can use interval-averaged gyro measurements to drive a complementary attitude estimate. The training variants include changes to brake timing, the reference head trajectory and feedback around that trajectory. Floor friction, the timestep, fixture contact and floor coordinates are unavailable to the controller.

Acquisition requires 25 consecutive control frames near the requested left edge with low roll rate and small pitch. Evaluation then requires another half-second of measured rim support at 28–35° lean. A circle requires a completed trial with joint clearance, a closed path of the **foot center** and at least 95% loaded rim contact during the lap. Median lean during the lap must be 30–32°, with at least 95% of frames at 29–33° and every frame at 28–34°. The floor fit uses the existing 350–385° sweep, radial consistency and endpoint tests. The report also records the unsmoothed center path’s mean diameter and endpoint gap for the same interval. All diameters are measured in the horizontal world plane. [Prior gyro-peg measurements](GYRO_PEG.md)

The four former failures are training conditions. Four separate evaluations change friction to 0.68, battery voltage and gain to 4.97 V and 0.995, the 0.03° noise seed to 5151, or the solver step to 1/9000 s. The 5.00 V nominal model was used to fit the orbit.

The early short screens scored captures that survived to the end of the trial. Final evaluation checks the additional half-second of measured rim support directly and scores the subsequent circle separately. Each finalist is compared on all geometrically closed windows that meet the lean and support requirements. Nine training commands evaluated 485 simulation rollouts, including reference recordings. They took 38.8 minutes elapsed and 160.7 CPU core-minutes on the M4 Mac mini. The system sensor measured 28.35 Wh gross, or 8.37 Wh above the sampled background. Gross electricity at an illustrative $0.30/kWh was $0.00850. The two noise-handoff fits overlap part of the endurance evaluation and video rendering. Gross energy covers the whole Mac during each fitting command. Process-tree CPU time in the receipts belongs to the trainer and its workers. [Compute ledger](../experiments/reports/training-compute.csv)

For an ideal circular rim of radius R rolling at a constant lean θ, with no sliding or twisting about the floor normal at contact, the horizontal foot-center diameter is 2R × (1/cos θ − cos θ). The 56 mm rim predicts about **3.47 cm** at 31°. The simulation report records the corresponding estimate at the measured lean and the integrated rotation about the floor normal during each fitted lap. Added twist, sliding or changing lean can change the radius. The comparison describes this form of rolling and gives context for the larger head-yaw experiments.

## Noisy roll after capture

The first combined controller acquired the edge on all four former failures. On the noisy case it fell 5.80 s into the roll episode. Filtering attitude against a nominal trace alone also failed. The final noisy branch uses a nominal 50 Hz head-motion demonstration as its motor reference and compares measured roll and pitch against the expected attitude. Outside the fitted 0.14° band, it blends in the live edge regulator at 0.35 per degree of excess error and rechecks IK. Phase aligns to measured edge capture. After the stored trajectory runs out it cycles the last reference period.

During the matched 36-second noisy run, all 1,742 reference frames stayed inside the band. The recorded trajectory carried this run without a residual correction. This demonstrates noise rejection with matched dynamics. A new friction, voltage or mass distribution can put the physical system outside that motion. The changed-dynamics results below test that limitation. [Frozen controller](../configs/deep-lift-circle.json) · [Individual trials and receipts](../experiments/reports/motor-training-deep-lift.json)

## Qualification from a flat start

| Evaluation group | Captured edge | Completed 36 s with a qualifying center lap |
| --- | ---: | ---: |
| Four former failures used for training | 4/4 | 4/4 |
| Nominal comparison | 1/1 | 1/1 |
| Four separate floor, voltage, noise-seed and timestep variations | 2/4 | 1/4 |
| Two additional noise seeds on nominal dynamics | 2/2 | 2/2 |
| New combined 0.69 friction, 4.99 V, reduced gain and attitude noise | 0/1 | 0/1 |

In the four separate variations, the new friction case reached a joint stop at 2.44 s, the new battery case failed to lift, and the finer timestep completed its circle. Noise seed 5151 captured the edge but reached a joint stop at 21.82 s after 34 live correction frames. The two additional passing noise seeds stayed within the demonstration band through the full episode. The combined test invoked the residual on 1,737 frames and missed the initial capture. The original noisy branch is qualified through its recorded 36-second trials. The 75-second endurance tests covered the nominal and reduced-voltage branches.

## Hanging ball

The 25 g ball is 5 cm across and hangs from a fixed point 73.5 cm above the simulated floor on a 62 cm string. Its center is 11.5 cm above the floor at rest. Eight inches (20.32 cm) is the horizontal gap between the upright robot's side shell and the near surface of the hanging ball at that height. A ray against the collision geometry places the ceiling anchor independently on each side. Left and right refer to the initial robot heading. Both anchors remain fixed in the room.

At five seconds a latch releases the stationary bob from its stated angle outward from vertical. The tether can pull taut or go slack. The free bob can rebound out of its original swing plane. Across all releases, anchor-to-bob distance ranged from 54.7 to 62.09 cm. The renderer draws a straight segment between them during slack intervals. Gravity, the cable and collision forces produce every hit. The test measures contact speed on the integration step before the first force-bearing robot contact. The solver continues after a joint warning or body landing so the video can show the outcome. A recovery requires a captured edge before release, an actual robot hit, joint clearance and two seconds after the last contact, with at least 95% of the final window on the loaded rim at 29–33° and the last frame in range. Late hits extend observation up to 20 seconds. The zero-degree release on each side is a control. A miss and a recovered hit are counted separately.

| Release from vertical | Left of starting heading | Right of starting heading |
| --- | --- | --- |
| 0° | Miss | Miss |
| 10° | Miss | Miss |
| 15° | Fell, 0.29 m/s | Miss |
| 20° | Fell, 0.64 m/s | Fell, 0.54 m/s |
| 25° | Fell, 0.90 m/s | Fell, 0.58 m/s |
| 30° | Fell, 1.14 m/s | Recovered, 0.37 m/s |
| 45° | Fell, 1.80 m/s | Settled flat, 1.44 m/s |

Nine of fourteen releases made contact. Reachy recovered on the right-side 30° release through two contacts, with incoming speeds of 0.37 and 0.17 m/s. It finished at 31.1° and kept the loaded edge throughout the final two seconds. Seven impact trials ended in body-floor contact and one settled onto the flat foot. The two 0° controls and three smaller swings missed. The robot rotates as the bob swings, so a larger release angle can change where in the arc they meet. The 20° right release first touched Reachy almost 12 seconds into the episode. Extending that run revealed the fall. [Watch the angle sweep](../media/simulation/deep-lift-pendulum.mp4)

## Replay

Use the project's pinned robot assets and simulation environment. The circle replay verifies its metrics against the archived run before rendering. The pendulum command regenerates all fourteen angle and side recordings.

```sh
.venv/bin/python experiments/train_deep_lift.py replay
.venv/bin/python experiments/film_gyro_peg.py experiments/generated/deep-lift/best.json --output experiments/generated/deep-lift/circle-replay.mp4
.venv/bin/python experiments/test_deep_pendulum.py --workers 6
.venv/bin/python experiments/test_deep_pendulum.py --bracket --workers 3
.venv/bin/python experiments/film_deep_pendulum.py --output experiments/generated/deep-lift/pendulum-replay.mp4
```

To repeat training, run each phase of the trainer in order. The archived power receipts record the exact command, worker count, time series and background measurement for each fit. Rebuilding the archive requires the corresponding metered receipts.

```sh
for phase in fit repair tune refine course refine-circle select-circle
do
  .venv/bin/python experiments/train_deep_lift.py "$phase" --workers 8
done
.venv/bin/python experiments/train_deep_lift.py qualify --workers 5
.venv/bin/python experiments/repair_deep_circle_noise.py fit --workers 5
.venv/bin/python experiments/repair_deep_circle_noise.py trajectory --workers 6
.venv/bin/python experiments/repair_deep_circle_noise.py verify --workers 5
.venv/bin/python experiments/test_deep_pendulum.py --workers 3
.venv/bin/python experiments/test_deep_pendulum.py --bracket --workers 3
```
