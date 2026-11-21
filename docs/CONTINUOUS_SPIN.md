# How Reachy Mini keeps turning

A single mirrored head policy turns the free base and torso from a flat, stationary start in MuJoCo. It has no target heading or reset between laps. The waist servo holds a zero-radian target throughout. The head moves on a repeating orbit in roll, pitch, yaw and translation. A one-second ramp brings it into the orbit.

## One-minute results

The frozen policy passed all 16 direction and floor combinations. None fell or reached a joint stop. The fastest held-out trial completed 6.08 left turns in real time. The base center stayed within 25.9 mm of startup, the peak tilt was 5.99°, and the base advanced during 99.86% of post-startup intervals. Its longest pause or reversal lasted 0.06 seconds. The same rim carried the robot for only 0.02 seconds in total under the stated load criterion. [Watch the run](../media/simulation/continuous-spin.mp4)

| Condition | Friction | Supply | Turns left in 60 s | Turns right in 60 s | Peak center travel, left / right |
| --- | ---: | ---: | ---: | ---: | ---: |
| Nominal | 0.700 | 5.00 V | 4.18 | 4.04 | 30.9 / 30.4 mm |
| Slick | 0.550 | 4.80 V | 3.78 | 3.66 | 32.7 / 32.9 mm |
| High grip | 0.850 | 5.35 V | 5.38 | 5.19 | 25.2 / 28.2 mm |
| Redistributed mass | 0.760 | 5.10 V | 5.99 | 5.84 | 24.9 / 26.2 mm |
| Held-out nominal | 0.685 | 4.96 V | 3.98 | 3.87 | 30.4 / 31.8 mm |
| Held-out slick | 0.565 | 4.82 V | 3.69 | 3.58 | 32.8 / 33.6 mm |
| Held-out high grip | 0.824 | 5.25 V | 5.01 | 4.83 | 37.2 / 36.8 mm |
| Held-out mass | 0.775 | 5.08 V | 6.08 | 5.93 | 25.9 / 24.9 mm |

With the head held at neutral from startup, the base accumulated less than 0.001° in 10 seconds. When the moving head target was held fixed at 12 seconds, the nominal and slick runs advanced just 4.00° and 2.65° farther over the next three seconds. After the first half-second of the hold, only 0.10° and 0.006° of net rotation remained. The repeating head motion supplies the drive for the spin.

The primary search used 910 simulated trials, including screening, cross-condition checks and finalist runs. Some exploratory candidates fell or overshot joint travel. The report preserves those counts by training stage and lists every frozen qualification result. [Frozen controller](../configs/continuous-spin.json) · [Full measurements](../experiments/reports/motor-training-continuous-spin.json)

## Three minutes without a reset

On the nominal floor the fast policy completed 12.68 left turns and 12.26 right turns in separate uninterrupted 180-second runs. Peak center travel was 32.7 mm and 30.4 mm respectively. Both runs met the same continuity and clearance criteria as the one-minute trials.

## A more tilted spin

A second search ranked eight candidates on measured base rotation during continuous same-rim loading. It tested each candidate for one minute in both directions on the nominal floor. The selected policy uses a 0.24-second orbit with about 26° each of commanded roll and pitch, a 22° yaw amplitude and an 11 mm vertical oscillation. Its timing is open-loop. [Rim policy](../configs/continuous-rim-spin.json)

On the reserved friction-0.685 floor this policy passed in both directions. The left run turned 2.80 times. A confirmed rim carried load for 44.9% of the minute, accounting for 516° of net base rotation. Peak tilt was 9.8°, and the base center stayed within 22.1 mm of startup. The right run completed 2.76 turns with 498° of net base rotation on a loaded rim. The robot alternates loaded edges throughout the orbit. The longest uninterrupted sequence of confirmed rim support in the featured left run lasted 0.24 seconds. [Watch the rim spin](../media/simulation/continuous-rim-spin.mp4)

The nominal left run also passed. On the nominal right run the robot stayed upright for the full minute and made 3.49 turns, but it traveled 63.3 mm from its start and advanced in only 61.4% of post-startup intervals. The fast policy passed both directions on that floor.

## The experiment

The CPU search varied the period, amplitudes, phase offsets, head height, lean and yaw braking waveform. It also tried changing the head cadence using a filtered estimate of the base yaw rate from noisy, delayed heading observations. Short trials screened for rotation speed, direction, center travel and joint clearance. Mirrored trials across surfaces tested whether the motion also worked in the other direction. One-minute trials decided among the finalists. The four additional floor and motor combinations were reserved until after the policy was frozen.

The selected policy uses a 0.20 s head orbit with open-loop timing. Mirroring roll, lateral translation and yaw reverses the commanded direction. The command waveform has nominal amplitudes of 40° roll, 26.1° pitch and 15.0° yaw before workspace projection and motor tracking. In the fastest film, actual head roll ranged from −18.0° to +18.7°, and the position-holding waist deflected between −1.2° and +1.5°. The position controller sends commands at 50 Hz. It uses inverse kinematics to shrink unreachable head poses toward neutral, keeping at least 0.08 radians of commanded joint margin. The actuator model then applies battery-dependent torque–speed curves, the existing 80% PWM limit, the 10.7 rad/s target speed cap, the 360 rad/s² target acceleration cap and motor delay. Simulated contacts and floor conditions enter evaluation. The controller never receives the floor coefficient, pivot point or contact forces. Hardware friction, inertia, voltage sag and heating during sustained use still need measurement. [Motor model](../configs/motors-speed-trained.json)

## How we count a spin

Base and torso headings are separately unwrapped across each revolution. A one-minute trial passes when both advance at least two full turns in the requested direction, the base center stays within its 56 mm radius of the starting point, and the base continues advancing in the second half. After a one-second startup, directed base speed must exceed 2°/s in at least 98% of overlapping half-second windows and in at least 80% of the individual 20 ms intervals. It must finish with positive or zero measured joint clearance and pass the simulator guards. There is no tilt-angle cutoff. An impact with the floor by a part other than the foot ends the trial.

For a confirmed rim load, one half of the foot must carry more than 0.5 N while the opposite half carries less than 0.1 N, with at least 10 mm of vertical separation along that axis. We count rotation on that rim only between consecutive samples that satisfy the criterion on the same side or on the same fore/aft edge. Center excursion is the largest distance from the starting center at any time during the run.

## Reproduce

From the project root with the project environment and pinned Reachy assets installed, replay and film the fastest qualifying held-out minute. The replay checks that every recorded metric matches the frozen qualification.

```sh
.venv/bin/python experiments/train_continuous_spin.py record
.venv/bin/python experiments/film_target_pivot.py experiments/generated/continuous-spin/best.json --output experiments/generated/continuous-spin/replay.mp4
```

To replay the higher-rim-load variant from the checked-in qualification report, run the following commands.

```sh
.venv/bin/python experiments/train_continuous_spin.py record_rim
.venv/bin/python experiments/film_target_pivot.py experiments/generated/continuous-spin/rim-best.json --output experiments/generated/continuous-spin/rim-replay.mp4
```

To rerun the seeded search and evaluations on CPU, run the stages in order.

```sh
for stage in screen evolve compare drive rim_search cross feedback refine lean long_probe freeze qualify sustain record rim_demo rim_record
do
  .venv/bin/python experiments/train_continuous_spin.py "$stage" --workers 8 || break
done
```
