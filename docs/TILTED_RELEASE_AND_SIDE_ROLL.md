# What changes when Reachy balances on the CAD foot?

## Why the deep roll looked surprising

Imagine holding a dinner plate at a fixed tilt, with one point of its rim on the table. Move the low point around the plate. Successive pieces of the rim take the load, and the plate center can travel much less than the contact point. In the earlier simulation the head continually adjusted the weight over that moving support. A freely spinning top would be a poor model for this slow movement. [Rolling disk kinematics](https://rotations.berkeley.edu/the-rolling-disk/)

In the 21.8-second filmed lap the horizontal distance between the modeled center of mass and floor contact had a median of **0.10 mm** and a maximum of **0.23 mm**, sampled at 50 Hz. The foot center made a 3.59 cm circle while contact traveled around a roughly 13 cm circle. From the same flat start, a repeat on the circular foot made 1.54 rim revolutions in 32 s. Holding the last head command from 10 s led to body-floor contact at **11.15 s**.

[Measured floor paths and center of mass in side view](../media/simulation/deep-roll-support.png)

The earlier model supplied a circular cylinder with a 56 mm radius. For this experiment the pinned Reachy part meshes provide the contact surfaces for the foot, turning ring, torso panels, head shells, lenses, and both antennas. The contact solver uses each mesh's convex envelope against the plane. The flat plane sees the outermost vertex along its normal. Other robot contacts, inertias, timing, and actuator limits are retained. The comparison changes the floor interface as a whole. The real underside, pads, friction, shell and antenna compliance, and battery mass distribution still need measurement. [Pinned robot geometry](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml) · [MuJoCo mesh contact](https://mujoco.readthedocs.io/en/stable/computation/)

Replaying the saved poses geometrically, the lowest CAD foot patch sometimes lay **23.0 mm horizontally** from the cylinder's contact. On the CAD scene the controller rocked to 18.4°, missed the initial capture and finished flat. Its modeled joints retained 0.047 rad of travel clearance. That tests the complete behavior from startup. Sustaining an orbit after a successful CAD-foot launch is still an open question. The deep roll belongs in the simulation experiments, outside the current hardware candidates.

## What counts as a catch

An external support is modeled by placing the assembled robot at the stated tilt with every velocity zero and its lowest shell vertex at floor height. The support is absent throughout the rollout. The head starts neutral. The 50 Hz regulator uses delayed roll, pitch, simulated gyro, and relative head pose. The scorer separately observes the contact forces, joint travel and motion. A pass requires the last **five continuous seconds** on one loaded foot edge, roll within 2° of the 32.1° target, absolute roll rate at most 5°/s, pitch within 5°, and joint clearance through the trial. The simulation continues through an inward return, a joint warning, or an outward landing. A settled flat base is a distinct outcome.

Across the initial nine release angles per side (24°, 28°, 30°, 31°, 32°, 33°, 34°, 36°, 40°), the previous regulator ended flat five times and landed on the shell eleven times. The two 32° releases kept the foot tilted for seven seconds but oscillated too quickly for a strict pass. With the head held still, ten releases settled flat and eight tipped onto the shell. The first contact on the 40° right release with feedback occurred at 60.8° lean, 0.304 s after release. The simulated mechanism also registered a travel warning in this trial. Its passive counterpart landed on the torso at 0.266 s with positive joint clearance. These are stationary releases. The earlier rocking launch and the separate forty-degree maneuver that returned to flat have different initial conditions and success criteria.

## Rolling on the torso

The robot starts at 90° on either side and settles under gravity before the head stroke. For the settled comparison, the head is prepared 18 mm away from the floor with 20° of head roll, mirrored across sides. Each four-cycle test starts after eight seconds. The last 0.2 s before the stroke had less than 0.006 rad/s of torso angular speed for the fixed prepared pose. The waist target stays zero. A matched control holds the same starting head pose. A second family steers the clearance bias along the estimated upward direction using one-frame delayed attitude; its control makes the same bias adjustment without the oscillation.

Floor contact is inspected every integration step (8 kHz nominal), including setup after release, the stroke, and the settling tail. Any solved head, lens or antenna touch disqualifies a head-clear attempt. Net torso rotation integrates the component of angular velocity about the body's long axis. Peak-to-peak rotation, movement of the foot center, contact migration, and tangential slip help distinguish a partial roll from a sustained barrel roll. All experiments use the bounded battery motor model and a fixed waist. Physical calibration and robot deployment are pending.

## What the trained release controller managed

A gain search, a local dynamics fit on each CAD edge, and a search over desired pitch were compared at the 32° development start. The local models used 48 short excitation rollouts; they did worse in closed-loop evaluation. On the left, the selected regulator used the previous dynamics model, gain 0.9, pitch-channel gain multiplier 0.7, a 2° pitch reference shift, and a 120-unit/s command limiter (degrees/s for angular commands, half as many mm/s for position commands). The right-side selection retained the original regulator. No controller receives ground contacts.

| Separate qualification | Ended balanced on the foot edge | Passed every strict criterion over the last 5 s |
| --- | ---: | ---: |
| Left, six new release, noise-seed or timestep settings | 6/6 | 1/6 |
| Right, six corresponding settings | 0/6 | 0/6 |
| 20 s nominal endurance, left and right 32° starts | 2/2 | 0/2 |

The right-side six all hit the body shell. On the left the successful **32.2°** release held strictly for **7.26 consecutive seconds** in an 8 s episode, finishing at 31.0° lean. Its last five seconds had 30.52–30.99° absolute roll and a maximum absolute roll rate of 4.85°/s. Its matched stationary-head control fell inward and stood flat. The other five left cases ended on their edges but exceeded the strict envelope during the final window. The 32.4° left case achieved a 7.18 s continuous strict interval earlier in its episode. The repeated 20 s nominal releases are reported separately from the six new cases per side. [Watch the catch, the passive control, and both failure directions](../media/simulation/cad-tilt-catches.mp4)

## How far the torso rolled with the head clear

The clean left-side example tilted the head fore–aft, turned it, and moved it fore–aft for four 1 Hz cycles. The torso rotated from −20.8° to +19.1° relative to the start of motion. The span was **39.9°**, and the final rotation was **−2.7°**. The nearest part of the torso surface migrated through **49.0°** around its circumference. Over the seven-second drive and settling window the foot center shifted 7.7 mm net, the estimated integrated torso slip was 5.8 mm, and only the torso touched the floor. The smallest sampled head clearance in that window was **9.7 mm**, and the integration-step contact audit detected no head touch anywhere in the 15 s trial. The corresponding still-head control turned less than 0.1°. A half-speed execution of the same head path produced an 8.2° span on that side. At the finer 0.1 ms integration step, the full-speed left-side result was 41.3°, again with no head touch. The partial rolling has a repeatable speed dependence in this simulated setup.

Doubling the head stroke produced an approximately 182° net rotation, but the front head shell touched at 10.278 s and head contact lasted 4.61 s in total. Steering the clearance bias with delayed attitude also produced large rotations, and every large rotation in that family included head contact. None demonstrated a sustained head-clear barrel roll. [Watch the still-head control, head-clear motion, and head-contact failure](../media/simulation/cad-side-roll.mp4)

The five metered fitting commands used **7.05 Wh gross** on the Apple M4 Mac mini CPU over 9.24 elapsed minutes and 27.18 CPU core-minutes, costing about $0.0021 at an illustrative $0.30/kWh. This is whole-Mac power during those commands, including concurrent evaluation work. No GPU or physical robot was used. [Per-run settings, failures, film data and power receipts](../experiments/reports/motor-training-shell-release.json)

## Reproduce

From the project root with the pinned assets and dependencies:

```sh
.venv/bin/python experiments/archive_shell_release.py replay --workers 4
.venv/bin/python experiments/train_deep_lift.py replay
.venv/bin/python experiments/audit_deep_shell.py
.venv/bin/python experiments/plot_deep_support.py
```

The film replay regenerates the selected physical trajectories, compares their measurements with the archive, and renders both reels under `experiments/generated/tilted-release/`. The development and qualification commands are:

```sh
.venv/bin/python experiments/tilted_release.py tilt --out experiments/generated/tilted-release/tilt.json
.venv/bin/python experiments/tilted_release.py fit --out experiments/generated/tilted-release/fit.json
.venv/bin/python experiments/fit_shell_release.py --workers 5
.venv/bin/python experiments/tilted_release.py local --out experiments/generated/tilted-release/local.json
.venv/bin/python experiments/tilted_release.py facet --out experiments/generated/tilted-release/facet.json
.venv/bin/python experiments/qualify_shell_release.py --workers 4
.venv/bin/python experiments/tilted_release.py deep --out experiments/generated/tilted-release/deep.json
.venv/bin/python experiments/tilted_release.py side_rest --out experiments/generated/tilted-release/side-rest.json
.venv/bin/python experiments/tilted_release.py follow_side --out experiments/generated/tilted-release/follow-side.json
```

For eventual hardware comparison, a supported stationary tilt and a measured contact patch on the user's actual surface would test the two quantities that mattered most here: available head authority and the location of support under load.
