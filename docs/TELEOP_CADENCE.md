# Walking at the cadence of the Quest demonstration

The operator estimated roughly one directional head swing per second and fewer than two per second. We count a leftward sweep and a rightward sweep as two swings. The estimate comes from the operator's recollection. The previous distance-per-swing controller recorded 160 swings in 24 active seconds, or 400 per minute. [Previous experiment](DISTANCE_PER_SWING.md)

## What the search learns

An evolutionary parameter search trains a timed head trajectory in MuJoCo. Its parameters set head roll, yaw and lateral translation, where the head sits fore–aft, the trajectory between endpoints and how sharply to brake. Roll, yaw and lateral motion can start and finish at different times so the head can move rapidly and stop while one rim is loaded. The waist motor holds its neutral target. This controller uses elapsed time and starting side. Motor position and velocity enter the simulated servo loop. Contact forces, base pose and head motion enter evaluation only.

The search tries 45, 60 and 75 scheduled directional swings per minute. The comparison score is forward millimetres per active second divided by the square root of measured swings per second. For equal forward travel, fewer strokes score higher. At the same cadence, more forward travel scores higher. The active clock includes holds between strokes and excludes the final 1.6-second settle. Distance is net forward displacement after settling. Each motion must make positive progress during the second half, settle flat and stay within 20° of the initial heading after every complete pair and at rest.

At 50 Hz we count directional excursions in actual relative head roll and yaw with 10° hysteresis and in lateral position with 5 mm hysteresis. We take the largest of the three counts and the scheduled count, allowing simultaneous motion across axes to count once. A significant rebound during settling counts against the walking budget. Each measured axis must have more than 0.50 seconds between events and at most three events in any two-second window. The full-trial limit is 78 counted swings per minute. Sub-threshold tremor is outside this definition.

The selector prefers passing trials on both starting sides, then repeated measured yaw braking on an edge, then the lower of the two cadence-weighted forward speeds. The edge criterion requires at least 10 mm vertical difference between the side rims, more than 0.5 N on one side and less than 0.1 N on the other. A braking event requires measured yaw speed above 20°/s and decreasing by over 100°/s². The trace also captures the head assembly's angular momentum about its own centre of mass, projected onto world vertical. Ground contact contributes an external impulse, so a measured reduction in head spin alone cannot establish how much momentum reached the torso.

The nominal simulation uses the existing 5 V battery assumption, XL330 head motor torque–speed model, 80% torque and PWM limits, 50 Hz targets, joint range checks and physical floor-contact termination. It has no 35° tilt cutoff. Friction, battery voltage, motor gains, controller delay and mass placement vary in the held-out cases. The robot's actual loaded bus voltage and contact behaviour still need measurement. [Motor model](../configs/motors-speed-trained.json)

## Nominal results

| Motion | Active time | Directional swings | Forward distance | Peak rim height difference | Swings braking on a loaded edge |
|---|---:|---:|---:|---:|---:|
| Selected forward gait, starting side −1 | 24 s | 24 | +70.97 mm | 5.26 mm | 0 |
| Selected forward gait, starting side +1 | 24 s | 24 | +70.90 mm | 5.21 mm | 0 |
| Deep rocking example, starting side +1 | 8 s | 8 | −30.66 mm | 13.73 mm | 5 |

All three runs start flat and finish upright. The selected forward gait made 2.96 mm per swing on the recorded side and gained 35.42 mm in its second half. Its largest pair-end heading error was 2.84°. The learned trajectory starts lateral motion about 69 ms into a stroke, roll at 79 ms and yaw at 151 ms. Its commanded yaw sweep takes 110 ms. The motor model limits the actual head, whose measured yaw speed peaked at 222°/s. The deep rocking trial reached 692°/s and spent 0.48 seconds on a strictly loaded edge. It moved backward in both halves. The two real-time clips show the sign of travel and label frames with simultaneous braking and edge support. [Forward walk](../media/simulation/teleop-cadence.mp4) · [Backward edge-braking example](../media/simulation/teleop-cadence-edge-example.mp4)

### Controls and held-out floors

At 40 active seconds, the two starts completed 40 swings and advanced 122.42 and 122.35 mm. Halving the nominal physics timestep from 0.25 to 0.125 ms gave 70.81 and 70.69 mm over 24 seconds. Both checks passed on both sides. The archive contains 1,031 search and evaluation trials, plus the separately recorded demonstrations. [All trial outcomes and policy parameters](../experiments/reports/motor-training-teleop-cadence-2026-11-20.json)

All ten held-out trials passed, comprising five independently sampled combinations of voltage, friction, motor gain, command delay and mass placement tested from both sides. They covered 45.43–58.54 mm in 16 swings and 16 active seconds, and none fell. The greatest rim difference was 9.01 mm. These conditions cover a limited simulation range.

Simply retiming the old walk to the same cadence produced 1.20–1.48 mm over the entire 24-second trial. Stretching the new stroke to 440 ms with softer braking produced 0.22 mm. Removing commanded yaw gave about 22.5 mm but introduced head reversals only 0.20 s apart and failed the cadence check. Changing only the brake fraction to 45% requested a pose outside inverse-kinematic joint limits from each side. That comparison provides no completed trial to isolate braking sharpness. The next experiment should trigger head yaw and braking from measured roll and roll rate and score forward displacement during each loaded-edge interval.

## Replay

From the project root with the project environment and upstream model installed, use a fresh output directory. Playback uses saved MuJoCo states. Each MP4 includes the final settling period.

```sh
.venv/bin/python experiments/train_cadence.py --replay configs/teleop-cadence-trained.json --side -1 --seconds 24 --out experiments/generated/teleop-replay
.venv/bin/python experiments/render_sway.py experiments/generated/teleop-replay/walk.json --output experiments/generated/teleop-replay/forward.mp4 --slowdown 1 --title 'REACHY MINI | TELEOPERATION CADENCE' --diagnostics --frame-path --follow-front
```

To reproduce candidate generation and selection, use a fresh training root. The first sweep uses an additive 3 mm per-stroke cost. The four refinement sweeps rank progress with the cadence-weighted score. Each search uses a fixed random seed. All five finalists face both nominal starting sides before selection. Held-out conditions do not enter the selector.

```sh
cadence_root=experiments/generated/teleop-retrain
.venv/bin/python experiments/train_cadence.py --out "$cadence_root/first"
.venv/bin/python experiments/refine_cadence.py --out "$cadence_root/impulse"
.venv/bin/python experiments/refine_cadence.py --reverse-seeds "$cadence_root/impulse" --out "$cadence_root/redirect"
.venv/bin/python experiments/fit_edge_brake_cadence.py --source "$cadence_root/impulse" --out "$cadence_root/lift"
.venv/bin/python experiments/fit_edge_brake_cadence.py --source "$cadence_root/impulse" --out "$cadence_root/forward" --forward-lift
.venv/bin/python experiments/select_cadence.py --root "$cadence_root" --output "$cadence_root/selected.json"
```
