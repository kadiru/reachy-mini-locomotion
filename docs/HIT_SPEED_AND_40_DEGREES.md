# How fast a hit can Reachy Mini withstand?

**Fastest recovered bounce with the roll-rate fallback.** A measured **9.248 m/s** ball struck the lower torso from the left while Reachy balanced on its left edge. It rebounded at **6.82 m/s**. The controller started returning the head to neutral **43 ms after impact**. Reachy met the sustained flat-settling criteria from **0.72 s after impact** to the end of the trial. On the same edge and controller, a **9.373 m/s** hit produced floor contact **0.366 s** after impact. [Watch both collisions at one-tenth speed, then both full trials](../media/simulation/fast-ball-catches.mp4).

In 230 original-controller MuJoCo ball trials, the fastest hit followed by sustained edge balance was **1.200 m/s**. Reachy started flat, lifted onto its right edge, took a ball from the right, tilted to **37.13°**, and regained qualifying edge support **2.43 s** after contact. On the same edge and seed, a **1.175 m/s** hit and a **1.205 m/s** hit both toppled the robot. Ball arrival time and the robot’s state at contact matter. [Watch the 37-second comparison](../media/simulation/ball-hit-speed-examples.mp4).

## Observed limits by direction

The following values are measured incoming speeds in m/s, rounded to three decimal places. Each row uses the same 5 s release, nominal sensor seed and 0.125 ms physics step. “First floor” means the lowest sampled speed that produced body contact with the floor after a qualifying pre-impact hold. Several speeds between recovery and floor contact triggered a joint limit, leaving their eventual floor outcome unknown.

| Supporting edge | Ball from | Fastest sustained edge recovery | Fastest finish upright | First observed floor contact |
|---|---|---:|---:|---:|
| Left | Left | 0.840 | 0.850 | 0.855 |
| Left | Right | 0.550 | 0.550 | 2.349 |
| Right | Left | 0.410 | 0.410 | 2.649 |
| Right | Right | 1.200 | 1.200 | 1.175 |

In the 1.200 and 1.205 m/s right/right trials the ball struck the same lower torso body. First contact occurred at 5.129875 and 5.129250 s respectively. Their integrated normal impulses were 0.03326 and 0.03318 N·s. The slightly faster ball actually delivered a slightly smaller normal impulse, yet only that run toppled. The outcome depends on the subsequent contact and controller dynamics.

With a different observation-noise seed, the left/left case toppled at **0.830 m/s**. With the nominal seed and a later release, it finished upright at **0.870 m/s** while missing the edge qualification. For right/right at 1.2 m/s, changing only the seed, only the release time, or both caused a joint stop. At the highest release speed, **6 m/s**, two directions produced floor contact and two reached joint limits. All four directions recovered at a nominal 0.4 m/s release. The data do not establish a guaranteed speed across all conditions.

Of the 230 runs, 211 struck a robot that already met the pre-impact balance criterion. Among those 211, **63 recovered on the edge**, **12 finished without meeting the full recovery criterion**, **29 contacted the floor** and **107 stopped on joint travel**. Of the remaining 19 runs, 10 ended before impact and nine lacked a qualifying pre-impact hold. Their physical outcomes are recorded separately. No ball trial that crossed 40° completed a qualifying edge recovery. [Speed chart](../media/simulation/ball-hit-speed-chart.png) · [All 230 trials on video](../media/simulation/ball-hit-speed-sweep.mp4) · [Per-trial CSV](../experiments/reports/motor-training-hit-speeds-2026-11-20.csv).

The point of impact changed consistently with direction. In all 114 eligible hits from the supporting side, the ball struck the lower torso. In all 97 from the raised side, the ball struck the foot. At a 6 m/s release, the two foot strikes generated peak normal forces of 121–127 N under the simulated contact law. These are force estimates for this geometry and compliance. The logged impulse sums normal-force magnitudes over contact, which can include load transferred through the floor.

The terminal joint angles identify which head actuator exhausted its travel in the 107 joint stops after a qualified hit. The most frequent was `stewart_4` at its lower limit (39), followed by `stewart_3` at its upper limit (30), `stewart_2` lower (21), `stewart_5` upper (15) and `stewart_1` upper (2). All ten pre-impact launch failures reached the upper limit of `stewart_5`. [Terminal joint angles and body state around each impact](../experiments/reports/motor-training-hit-state-details-2026-11-20.csv).

## Giving up the edge to stay upright

The fallback arms after half a second of sensed edge balance. When absolute measured roll rate reaches 10°/s, it moves the head toward neutral over 0.2 s. It uses the existing 50 Hz feedback and bounded servos. Its inputs contain neither ball position nor impact time. The objective here is to settle flat and avoid a topple. The 1.200 m/s result above retains the edge.

| Supporting edge and incoming side | Fastest nominal bounce with a flat recovery | First nearby confirmed topple | Time from impact to sustained flat settling |
|---|---:|---:|---:|
| Left edge, ball from left | 9.248 m/s | 9.373 m/s | 0.72 s |
| Right edge, ball from right | 8.873 m/s | 9.373 m/s | 0.42 s |

On the right edge the 9.0 m/s release reached a joint limit before the controller issued its return. At 6 m/s, the same roll-rate policy on the same right edge settled flat after a hit from the right and touched the floor 0.158 s after a hit from the left. Contact location is important. Both top recoveries reproduced with the second sensor seed, the later release and both changes together at the nominal physics step. At 0.1 ms some runs lose qualified balance before the ball arrives.

All eight unstruck controls held the edge without invoking the return. Across 247 fallback trials and controls, there were 71 settled-flat outcomes after a bounce, 18 flat finishes without rebound, 76 floor contacts, 58 travel stops, five infeasible head commands, five original 80° stops, eight unstruck controls and six unqualified pre-hit starts. The 76 floor contacts include five follow-through runs of the original 80° stops. These are counts of trials, including deliberate comparisons.

The high-speed sweep extends to 12 m/s. The fastest measured upright finish was 11.999 m/s from the supporting side, with no bounce and a 2.37 kN peak simulated normal force. At 10–12 m/s, some runs ended flat while the ball continued past the robot, without a qualifying rebound. The right-sided 10 m/s case contacted both the torso and foot and produced a simulated peak normal force above 2 kN. The force and rebound behavior in that regime need measured contact calibration. The highest upright finish and the highest recovered **bounce** are different observations.

Five early fallback trials reached the old 80° diagnostic backstop. All five were replayed with tilt-only termination disabled, keeping the other guards active. Every continuation reached actual floor contact, 0.125–2.125 ms after the old stop. New balance trials use the contact-only rule by default. [All emergency-return trials on video](../media/simulation/ball-emergency-returns.mp4) · [Per-trial conditions, physical contacts, motor bounds and outcomes](../experiments/reports/motor-training-impact-returns-2026-11-20.json).

## How the ball tests work

The frozen side controllers lift the robot naturally. The simulation releases an 8 cm diameter, 10 g ball 30 cm from the base center with rolling angular velocity. Gravity and contact dynamics produce the collision. The controller receives its usual sensor inputs. Incoming speed is the horizontal speed of the ball center at the last physics sample before its first measured robot contact.

The 14 s trials use release times of 5 or 7 s, observation-noise seeds 2026112101 or 2026112231, and physics steps of 0.125 or 0.1 ms. The sampling is denser near observed transitions. A later release leaves two fewer seconds before the final scoring window. A qualified recovery requires a pre-impact hold, a rebound, a continuous half-second of strict balance within three seconds of contact, and strict balance over the final five seconds. Strict side balance requires support on one edge, roll within 2° of target, roll rate within 5°/s and pitch within 5°. The reported recovery time is the start of the qualifying half-second.

The balance evaluators record 35° and 40° crossings and continue. A non-foot contact with the floor above 0.5 N normal force counts as a topple. Joint travel, infeasible head commands and mechanism errors retain their own termination reasons. New balance runs continue until the time horizon or a physical or numerical stop, including beyond 80°. The 230-run baseline and the original raised-start study used the earlier 80° diagnostic backstop. None of those 970 trials reached it. Earlier walking screens and archived films have their original criteria. All experiments here use the zero waist servo target and existing battery motor model, including its torque, speed and acceleration limits. The model assumes a 5 V bus. Actual bus voltage, mass distribution, contact response and IMU behavior still need hardware measurements.

## Can it recover after measuring 40°?

**Yes, from two prepared, outward-moving states in simulation.** A bounded head movement returned Reachy to flat even when commanded only after the one-tick-delayed roll reading reached 40°. Each control run started with identical joint positions and velocities and held the prepared head pose. Each comparison reproduced at both 0.125 and 0.1 ms physics steps.

| Supporting edge | Initial tilt and outward rate | First 40° crossing | First catch command | Peak with catch | Control hit floor at |
|---|---|---:|---:|---:|---:|
| Left | 39.95°, 3°/s | 18.6 ms | 40 ms | 40.45° | 72.84° |
| Right | 39.99°, 2°/s | 8.8 ms | 40 ms | 40.36° | 72.51° |

The catch is sensitive to timing. A right-side start at 39.98° with the same 2°/s outward rate crossed the sensor threshold one control tick later and toppled. Starting on the left at 40° and 10°/s outward also toppled, even with the earlier catch. The body was already rolling outward at about 47°/s at the first control sample after the 40° crossing in the 0.855 m/s left/left topple, and at 53°/s in the 1.205 m/s right/right topple. Those natural disturbances approached 40° much faster than the prepared catches. The sustained edge policy alone failed all 18 raised-start tests at 38°, 40° and 42°. [Measured-threshold catches and a failure, in slow motion](../media/simulation/measured-40-catches.mp4) · [Earlier catches and a faster outward fall, in real time](../media/simulation/high-tilt-catches.mp4).

For this separate diagnostic, the head servos first move during a 0.5 s warmup on the floor. The simulation then places the base on its rim with a prescribed angle and angular velocity, matching the translation to keep that rim momentarily stationary. The selected maneuver starts with head roll 40° away from the loaded side, a 25 mm lateral offset toward the raised side and a 12 mm lower head height. Signs mirror between sides. The controller commands two successive roll/lateral targets, then returns the head to neutral as the body comes inward through 26°. It samples roll at 50 Hz with 20 ms delay and zero added noise. The outcome is settling flat. The ball study used the original noisy edge controller throughout.

Across 722 maneuver search and comparison trials, 134 settled flat, 587 made floor contact and one commanded an infeasible head pose during motion. These counts include matched controls, repeats at a second timestep and intentionally difficult starts. [All 740 raised-start trials, including the 18 original-controller checks](../media/simulation/raised-tilt-all.mp4) · [Inputs, outcomes and matched state checks](../experiments/reports/motor-training-hit-speeds-2026-11-20.json).

## Finding a particular trial

The 28:02 combined review contains 1,217 simulation trials and controls in 412 chapters. It starts with the 230 original edge-controller impacts, then the 247 fallback and control runs, and ends with all 740 raised-start trials. Its searchable index gives the chapter time, panel, input condition, trial ID and outcome. The 1.200 m/s edge recovery appears at 3:27, the 9.248 m/s return to flat at 18:10, and the measured 40° left-edge catch at 27:38. The three individual section films are also available above. [Search the complete video index](../experiments/reports/motor-training-hit-video-index-2026-11-20.csv).

[Watch the complete 1,217-trial film](../media/simulation/balance-stress-all-1217.mp4).

To assemble the single film from the three published MP4s, this command copies the compressed video without re-encoding it and generates a matching index.

```sh
.venv/bin/python experiments/join_hit_films.py --work experiments/generated/join-all-new --output experiments/generated/join-all-new/all-tests.mp4 --index experiments/generated/join-all-new/test-index.csv
```

## Replay

Use the project Python and pinned upstream model from the repository README. Run these commands from the project root and choose fresh output directories.

```sh
.venv/bin/python experiments/run_balance_curriculum.py configs/balance-recovery-experimental.json --edge right --mode single-hit --ball-from right --ball-speed 1.2 --release-at 5 --seconds 14 --record --output experiments/generated/right-hit-120.json
.venv/bin/python experiments/run_balance_curriculum.py configs/balance-recovery-experimental.json --edge right --mode single-hit --ball-from right --ball-speed 1.205 --release-at 5 --seconds 14 --record --output experiments/generated/right-hit-1205.json
```

Use `--sensor-seed`, `--release-at` and `--timestep` for the other conditions. To reproduce both full films, including terminal states, regenerate and verify the trajectories against the published report.

```sh
study=experiments/reports/motor-training-hit-speeds-2026-11-20.json
.venv/bin/python experiments/summarize_hit_envelope.py --report "$study" --replay --work experiments/generated/ball-replay --state-details experiments/generated/ball-replay/joint-details.csv
.venv/bin/python experiments/film_hit_envelope.py experiments/generated/ball-replay/hit-film.json experiments/generated/ball-replay/all-balls.mp4 --work experiments/generated/ball-replay/clips --speed 2
.venv/bin/python experiments/replay_raised_study.py "$study" --work experiments/generated/raised-replay
.venv/bin/python experiments/film_hit_envelope.py experiments/generated/raised-replay/raised-film.json experiments/generated/raised-replay/all-raised.mp4 --work experiments/generated/raised-replay/clips --speed 2 --fps 12
```

Individual raised-start maneuvers can be replayed by passing a JSON list containing their published `case` objects to `experiments/probe_high_tilt.py --jobs FILE --output NEW.jsonl`. Add a new `record` filename to a case to save its physical trajectory.

To replay the fastest nominal bounce, including high-rate collision poses, use the same frozen edge policy with the rate-based return enabled. The adjacent topple uses `--ball-speed 9.375`. To reproduce the full return film use the published outcome report.

```sh
.venv/bin/python experiments/run_balance_curriculum.py configs/balance-recovery-experimental.json --edge left --mode single-hit --ball-from left --ball-speed 9.25 --release-at 5 --seconds 14 --emergency-return 0 0 0 --emergency-rate 10 --record --collision-window 5 5.8 --output experiments/generated/left-925-dense.json
.venv/bin/python experiments/summarize_impact_returns.py --report experiments/reports/motor-training-impact-returns-2026-11-20.json --replay experiments/generated/return-replay --manifest experiments/generated/return-replay/film.json
.venv/bin/python experiments/film_hit_envelope.py experiments/generated/return-replay/film.json experiments/generated/return-replay/all-returns.mp4 --work experiments/generated/return-replay/clips --speed 2 --fps 12
```
