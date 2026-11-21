# Four strokes for a slick-floor quarter-turn

Reachy Mini starts flat in MuJoCo. The waist servo holds a constant zero target while head roll, yaw, translation and braking lift a rim and turn the base. The previous policy took eight strokes for the right quarter-turn on the matched slick floor (sliding friction 0.55, simulated 4.80 V battery supply). This experiment also trains on the three failed cases from the previous qualification.

## Matched results

| Condition and command | Strokes, previous → new | Measured head swings, previous → new | Settled torso, previous → new | New base turn on rim | Peak center travel, previous → new |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original slick, 90° right | 8 → 4 | 42 → 37 | 89.9° → 86.8° | 55.2° | 36.5 → 36.1 mm |
| Neighboring slick, 90° right | 9 → 5 | 67 → 58 | 91.1° → 90.6° | 49.1° | 64.6 → 40.9 mm |
| Neighboring slick, 90° left | 9 → 8 | 67 → 74 | 74.5° → 90.4° | 38.0° | 46.1 → 49.2 mm |
| Intermediate, 24° right | 3 → 2 | 20 → 15 | 22.9° → 27.4° | 16.2° | 19.3 → 16.0 mm |
| Friction 0.60, 90° right | 9 → 5 | 45 → 42 | 89.2° → 90.0° | 54.2° | 43.5 → 26.4 mm |

On the original floor, elapsed time fell from 36.0 to 18.4 seconds. Signed torso rotation while one rim was loaded rose from 1.71° to 2.03° per measured head swing. The final policy collected 75.1° of net torso and 55.2° of net base rotation during those contacts. The fixed waist target was 0 radians. Simulated compliance let the waist joint deflect from -2.5° to 1.9°. The smallest joint clearance during this turn was 0.054 radians.

The frozen policy passed **95/95** matched cases and **7/10** held-out cases. The previous eight-stroke policy passed 92/95 and 6/10 respectively. On the intermediate set, the initial fit passed 11/14 and fell once on a left turn. After training on the three misses, the final fit passed 14/14 with 0 falls and 0 joint stops. The matched suite recorded 0 falls and 0 joint-limit stops. The held-out suite recorded 2 falls and 0 joint-limit stops. The three formerly held-out failures are now training cases.

## Independent checks

The final ten tests were reserved before the repair search. One repeats the original slick floor with a new sensor seed. Nine test three requested headings on three new combinations of friction, voltage, motor gain, timestep and noise. The two slick surfaces use friction coefficients 0.554 and 0.573. The intermediate surface uses 0.702. Positive headings turn left and negative headings turn right.

| Condition | Command | Strokes | Settled or last torso heading | Result |
| --- | ---: | ---: | ---: | --- |
| check_slick_a | -90° | 9 | -87.0° | Pass |
| check_slick_a | -24° | 4 | -24.0° | Pass |
| check_slick_a | +90° | 5 | +82.5° | nonfoot floor contact, unsettled, heading error 51.0°, center excursion 71.2 mm, base turn on rim 16.4° |
| check_slick_b | -90° | 2 | -55.7° | nonfoot floor contact, unsettled, heading error 77.0°, base turn on rim 15.3° |
| check_slick_b | -24° | 2 | -20.9° | Pass |
| check_slick_b | +90° | 9 | +90.9° | Pass |
| check_middle | -90° | 4 | -90.1° | Pass |
| check_middle | -24° | 3 | -24.3° | base turn on rim 9.3° |
| check_middle | +90° | 5 | +89.4° | Pass |
| check_seed | -90° | 4 | -88.3° | Pass |

With exactly the original floor, battery and motor configuration, 3/3 additional evaluation noise seeds completed the right turn in four strokes. Their settled right headings were 87.7°, 87.9° and 88.3°. The video shows the most accurate qualifying four-stroke evaluation and the original matched run, each at real time and with the strongest contact replayed at half speed.

The first search used 1,348 training rollouts before comparing the fitted controllers. The repair search tried 142 more trajectories, then compared observation-matching thresholds. During the first search, rejected candidates produced 419 falls. The deployed policy contains 11 additional strokes and 4 heading/observation-delay groups. Other commands use the previous policy.

[Watch the frozen-policy replay](../media/simulation/slick-four-stroke-turn.mp4) · [Policy](../configs/slick-quarter-pivot.json) · [All results](../experiments/reports/motor-training-slick-quarter.json)

## What the controller learns

The search first tried stronger head motions on the original slick surface. Several could make a quarter-turn in four strokes there but tipped over on neighboring surfaces. Training then varied the preload, head roll, fore–aft position, rocking frequency, catch and yaw timing. It tested proposed follow-up strokes after a real opening stroke, carrying the robot, actuator and contact state through every landing. It also tested roll-triggered catches and mirrored actions for left turns. A four-stroke candidate that landed within 0.15° of the target toppled on other sensor-noise seeds. The next fit selected the third and fourth strokes from sequences that settled on all three training seeds. It then compared motion choices after a conservative opening on an ambiguous left-turn landing.

The resulting policy starts with a common stroke for a heading range and known observation delay. After each replant it compares the entire sequence of observed heading changes and peak rolls with successful training trajectories. A continuation must match the action history, the landing measurements and the current heading error. An unfamiliar landing returns control to the prior heading-feedback policy. The policy never receives the floor coefficient, contact forces, center position or noise seed as observations. Training and evaluation use those quantities to measure performance. All the training and evaluation took place in simulation.

## Motion and motor limits

A stroke includes a launch, rocking motion and return to neutral. Each can include several individual head swings. The swing counter uses actual head roll and yaw reversals with 10° hysteresis. The larger count of the two axes avoids double-counting simultaneous roll and yaw.

The same battery-dependent motor torque–speed model, PWM limit, joint ranges, target speed and acceleration limits, 50 Hz commands and actuator delays apply to both policies. The motor and contact models still need hardware calibration. Trials run through large tilts, with termination when a part other than the base strikes the floor or a guard trips. The base and torso must both settle within 5° for the final 0.6 s. Both must accumulate at least 40% of the commanded turn on a loaded rim. The maximum center excursion is 56 mm and the mean rim slip allowance is 15 mm per stroke. A loaded rim requires at least 10 mm of height difference, more than 0.5 N at one side, and less than 0.1 N at the other.

## Reproduce

From the project environment, with the pinned upstream Reachy assets installed, replay the frozen policy and render the full episode plus the half-speed contact replay.

```sh
.venv/bin/python experiments/train_slick_quarter_turn.py replay --condition slick --goal -90 --out experiments/generated/slick-quarter/replay.json
.venv/bin/python experiments/film_target_pivot.py experiments/generated/slick-quarter/replay.json --slow-edge --output experiments/generated/slick-quarter/replay.mp4
```

To repeat training and qualification on CPU, run the stages in order.

```sh
for stage in baseline search probe sample catches power left left_starts left_quick four finish_four stable_third stable_four precise_four stable_trim projected complete_right complete_left practice_right practice_left left_repeats fit paired qualification repair_search repair_radius repair_finish repair_left repair_replicas repair_fit paired_final repair_qualify film
do
  .venv/bin/python experiments/train_slick_quarter_turn.py "$stage" --workers 6 || break
done
```
