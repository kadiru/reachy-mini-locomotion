# Turning the torso while balancing on one foot edge

The foot starts flat. The six head actuators lift either the left or right side and arrest the fall. Once the measured roll and roll rate have stayed near the target for half a second, the waist motor begins a periodic sweep. The learned edge regulator continues to move the head in the direction required by the supporting foot. The torso can change heading while that support stays on the same material sector of the foot.

The official robot model has a bounded body-yaw hinge. From neutral it can turn 160° in either direction, for 320° between endpoints. The stock interface therefore requires reversals. A unidirectional world-heading turn extending across successive waist sweeps would require the foot to rotate against the floor. The experiments measure waist-encoder angle, world torso heading, and world foot heading independently. [Pinned robot model](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml#L74) · [SDK limits](https://huggingface.co/docs/reachy_mini/v1.9.0rc1/en/SDK/core-concept)

## The working sweep

In separate 125-second runs, the robot completed **10 full sweeps on each edge**. The original rim sector stayed continuously loaded for **121.84 seconds on the left** and **121.64 seconds on the right** from the start of waist motion, with every physics step passing the contact audit.

The recorded comparison trials completed three 12-second cycles from a natural capture, with the waist commanded through ±90°. The camera overlay shows the measured waist angle and the separate headings of the torso and foot. The world torso heading is the heading change since startup, including foot motion and tilt projection. [Watch both edges](../media/simulation/active-waist-edge-sweep.mp4) · [Frozen sweep parameters](../configs/waist-edge-sweep.json)

| Supporting edge | Capture time | Unbroken edge support after capture | Measured waist range | World torso heading range | Body lean magnitude | Largest departure from rim midpoint |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Left | 3.16 s | 37.84 s | −89.7° to +89.5° | −89.9° to +88.8° | 30.2°–34.0° | 5.67° |
| Right | 3.36 s | 37.64 s | −89.5° to +89.6° | −88.5° to +90.0° | 30.1°–33.6° | 3.55° |

Every one of the 302,720 left and 301,120 right integration steps after capture met the same-edge contact test. The foot yaw spanned about 14° in each trial. In floor coordinates, the sampled center of pressure stayed within 4.9 mm of its location when the sweep began on either side. The largest requested head-to-body yaw difference was 1.43°, and the largest requested head roll was 37.24°. The smallest joint clearance was 0.048 radians on the left and 0.055 radians on the right. Peak waist torque, over the entire rollout, was 0.051 and 0.049 N·m respectively. Wider candidates up to ±155° were tested. Several reached the requested extreme on the encoder and subsequently fell or hit a joint limit.

## What the head contributes

With the same launch and waist trajectory, fixing the six head-joint targets at capture lost the left edge after 0.90 seconds and the right after 0.96 seconds. Keeping balance feedback active but expressing its corrections in the rotating torso frame lost the edges after 1.14 and 1.32 seconds. The former trial fell and the latter reached a joint limit. All times here are measured from the start of the waist command.

A separate test used ±55° waist sweeps with a slow outbound stroke and a three-times-faster return. Both edges and both initial directions held for three cycles. The first cycle changed world torso heading by 6.9°–8.2° in magnitude, including startup and actuator lag. Subsequent cycles accumulated less than 0.08° each. This waveform produced oscillation with little sustained one-way rotation.

## Where it failed

The fitted ±90° policy passed **1 of 8** tests on four reserved changes. Seven trials never reached the stable edge state that permits waist motion. Reversing the initial sweep direction on the nominal setup passed on the left. On the right, Reachy lost the edge after 2.68 seconds and fell.

| Reserved change | Left edge | Right edge |
| --- | --- | --- |
| Sliding friction 0.67 instead of 0.70 | No capture | No capture |
| 4.95 V supply and 0.99 motor gain | No capture | No capture |
| 0.03° attitude noise | Fell during launch | Joint limit during launch |
| Finer 0.10 ms integration | Joint limit during launch | Passed |

The search included 52 short candidate rollouts, followed by six three-cycle endurance checks. The qualification report preserves the failed wide-angle candidates, reversed starts, control trials and the asymmetric stroke results. [Full measurements](../experiments/reports/motor-training-waist-balance.json)

## Control and measurement

The simulation uses the project's previously identified 20-state balancing model and its side-specific natural launches. The regulator receives simulated foot attitude and reconstructed head pose at 50 Hz. A waist encoder supplies the torso angle relative to the foot. During the sweep, the head follows the waist in yaw while its requested lean and lateral shifts stay aligned with the foot. The analytical inverse kinematics solve the seven commanded joints together. The CPU parameter search varies sweep amplitude, period, the change in feedback gain at large waist angles, and a yaw-dependent correction for a rotating fore–aft mass offset.

The free base receives no applied forces. The waist and head use the existing battery motor approximation, speed and acceleration caps, joint ranges, and PWM/torque limits. The waist torque–speed curve is a proxy. The supply, contact, onboard sensing and motor behavior have yet to be calibrated on the physical robot. [Motor model](../configs/motors-speed-trained.json)

After capture, the contacted sector must remain within 15° of the left or right midpoint of the circular foot, expressed in the foot's own frame. The opposite edge must be at least 10 mm higher, the requested side must carry more than 0.5 N, and the opposite side less than 0.1 N. The endurance and challenge runs audit all three conditions at every physics integration step. The contact point may move within that sector. In the video inset, the orange arc marks the allowed sector and the blue dot shows the force-weighted contact center.

A completed sweep must visit at least 85% of both commanded waist extrema, produce a world torso heading span at least as large as the commanded one-sided amplitude, and return through a full command period. The body roll must stay within 6° of the signed 32.1° target for at least 95% of active 50 Hz samples. Falling, crossing a joint limit, losing the sector, or a missed excursion fails the trial. The simulation continues past high tilt until physical floor contact or a model guard.

## Reproduce

Run from the project environment with the pinned upstream assets. Replay the selected three-cycle runs and render their launch and first complete sweep on each side.

```sh
.venv/bin/python experiments/train_waist_balance.py record
.venv/bin/python experiments/film_target_pivot.py --first-cycle experiments/generated/waist-balance/left.json experiments/generated/waist-balance/right.json --output experiments/generated/waist-balance/replay.mp4
```

To repeat the CPU search and evaluation, run the stages in order.

```sh
for stage in screen fit fit_offsets baseline_long endurance sustained qualify record summarize
do
  .venv/bin/python experiments/train_waist_balance.py "$stage" --workers 8 || break
done
```
