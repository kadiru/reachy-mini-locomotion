# Balancing while the floor tilts

Both experiments begin with a supported 32.2° sideways release on a level table. Reachy has three seconds to catch its chosen edge before the table moves. The previously fitted CAD-foot release policy runs at 50 Hz with a fixed neutral waist target. Its inputs are delayed, noisy body attitude and angular velocity, and measured head state. The controller sees neither the table command nor the foot contact forces. The result applies to an established balance. The [fixed-ramp launch experiment](SLOPE_EDGE_BALANCE.md) measures whether Reachy can get onto the edge by itself.

## Slow tilt and return

| Loaded rim | Highest tested tilt held on all three runs | Next tilt | Runs holding at next tilt |
|---|---:|---:|---:|
| Left, uphill | 5° | 5.62° | 2/3 |
| Left, downhill | 15° | 15.62° | 2/3 |
| Right, uphill | 28.75° | 29.38° | 0/3 |
| Right, downhill | 10° | 10.62° | 2/3 |

The 5.62° uphill-left miss landed on the shell during the dwell. The 15.62° downhill-left miss landed during the return, with 15.44° of bank remaining. At 10.62° downhill-right the missed run lost contact on the return at 9.15° and settled flat. All three 29.38° uphill-right trials landed on the shell near peak bank.

**Sliding matters.** On the uphill right edge, 20° was the highest sampled angle where all three also satisfied the endpoint steadiness limits. At 28.75° all three stayed on the rim, and all three missed the steadiness test. In the filmed trial the foot center traveled 52.9 cm laterally during the full excursion, including 1.18 cm of three-dimensional displacement in the last half-second of the uphill dwell. Body roll varied by only 0.15° during that half-second. The table physically moved underneath a robot that remained nearly aligned to gravity.

[Watch the tilt limits and actual landings](../media/simulation/moving-table-angle.mp4) · [Per-trial force, table and outcome measurements](../experiments/reports/moving-table.json)

## How far can it tilt while staying in place?

For 78 slow trials we measured the force-weighted contact point in coordinates attached to the deck at 50 Hz. Its starting location is the median of the last half-second before the deck moves. For this score we allow **10 mm maximum travel over the entire excursion** and require an uninterrupted edge hold. Travel can include both rolling along the CAD rim and sliding across the deck. All six level controls stayed within 4.9 mm.

| Loaded rim | Highest tilt with 3/3 inside 10 mm | First miss | Runs inside 10 mm at first miss |
|---|---:|---:|---:|
| Left, uphill | 3.5° | 4.25° | 1/3 |
| Left, downhill | 3.5° | 4.25° | 2/3 |
| Right, uphill | 4.25° | 5° | 0/3 |
| Right, downhill | 4.25° | 5° | 0/3 |

At 10° downhill-right, the contact traveled **45.9 mm** in each of the three successful edge holds. At 28.75° uphill-right it traveled **529.2 mm**. These measurements explain why the steep-slope video can show a nearly constant torso angle while the robot travels across the floor. [Per-seed footprint measurements](../experiments/reports/moving-table-footprint.json)

## How fast can the deck move?

Two full excursions at the slow starting speed first had to meet both contact and steadiness criteria in all three repeats. The uphill left edge qualified at 2° of travel. The other three orientations qualified at 5°. The two speed columns below are the requested constant cruise rates on each leg.

| Loaded rim | Excursion | Fastest 3/3 uninterrupted | First speed with a miss | Uninterrupted at first miss | Finished on edge at first miss |
|---|---:|---:|---:|---:|---:|
| Left, uphill | 2° | 152.22°/s | 158.96°/s | 0/3 | 2/3 |
| Left, downhill | 5° | 133.67°/s | 139.59°/s | 2/3 | 2/3 |
| Right, uphill | 5° | 166.00°/s | 173.35°/s | 0/3 | 3/3 |
| Right, downhill | 5° | 152.22°/s | 158.96°/s | 1/3 | 3/3 |

The uphill right edge completed every 166°/s excursion. At 173.35°/s all three runs briefly lost that edge and all three recovered it by the end. At 181.02°/s the same three also recovered. At 256°/s all three settled flat. At the uphill-left first miss, two recovered and one landed on the shell. The downhill-left 133.67°/s runs all held the edge, with only one meeting the stricter endpoint steadiness test. The repeated left-uphill 5° slow test itself had a shell landing, which is why that speed sweep used the smaller excursion.

The fastest legs last only tens of milliseconds. Inertia and contact dynamics contribute to those outcomes, and a 50 Hz controller receives few new observations during a leg. The feedback comparison below tests the contribution of new head corrections during the slower excursion.

[Watch the repeated speed trials and slow-motion replays](../media/simulation/moving-table-rate.mp4)

## Does head feedback make a difference?

On the downhill right edge we ran the same 10° excursion twice for each of three sensor-noise seeds. The live controller held the edge through the return in all three runs. In the paired run we sent the head pose sequence captured during a level-table trial, preserving every commanded head motion and the same motor dynamics. Before the table moved the commands matched exactly. With the prerecorded sequence Reachy lost its edge at 1.037–1.040° and eventually settled flat in every trial. On the moving table, live feedback changed the head orientation command by up to 4.0–8.9° and the position command by up to 0.50–1.11 mm across the three runs. This directly tests whether responding to the disturbance matters for this example.

[Watch both runs side by side](../media/simulation/moving-table-feedback.mp4) · [Three matched controls](../experiments/reports/moving-table-feedback.json)

## What moves

A physical hinge runs fore–aft underneath the initial center of the foot. Its surface rises or falls under the loaded rim as it rotates. Only the hinge receives the table drive. Gravity stays vertical and Reachy is free to tip, slide, yaw or fall onto its CAD shell. Floor friction is 0.7, physics runs at 8 kHz, and the head motors retain the project's battery torque–speed and travel limits. The 5 V bus and friction are simulation assumptions. The camera stays level. Body roll is measured relative to gravity and head roll is measured relative to the foot frame.

1. **Angle experiment.** The table rises smoothly from level, with a peak rate of 2°/s. It dwells at the target for one second, returns to level and gives Reachy two seconds there. We test both anatomical edges with their loaded rim uphill or downhill, including 10° in every orientation. Three sensor-noise seeds share each condition. We refine the first all-pass to mixed/fail interval to within about one degree.
2. **Speed experiment.** Each eligible orientation first completes two slow excursions, including the endpoint steadiness check. Keeping that amplitude fixed, we repeat two cycles at increasing speeds. Each leg spends 20% of its time accelerating, 60% at the specified angular speed and 20% decelerating. The center 75% of the angular travel occurs at constant speed. There is a one-second dwell at each reversal and two seconds at the end. We verify the measured peak speed to within 3%. As the speed increases the acceleration increases too, so the boundary belongs to this motion profile.

At the pass and first-miss settings we also run a level table for exactly the same duration with the same noise seed. A high-angle run takes longer than a low-angle run at the same speed. These controls check whether the regulator loses balance simply because it has been running longer.

## What counts as holding the edge

We sample foot forces at 1 kHz. The loaded side must carry more than 0.5 N, the opposite side less than 0.1 N, and the center of pressure must project at least 40 mm toward the requested side from the foot center. Opposite rim heights must differ in the requested direction by at least 10 mm measured normal to the deck. A gap lasting 20 ms (one control interval) counts as loss of the original edge even if Reachy later recovers. We record shorter gaps too.

An edge pass requires a steady start, continuous support under this definition, the correct final edge, full completion, table angle tracking within 0.1° at the 50 Hz checkpoints, and no shell contact or modeled travel violation. A second score checks drift during each hold. In the starting and final one-second windows and final half-second of each intermediate dwell, roll and pitch must each span at most 3°, yaw at most 5°, and foot-center displacement from the window start at most 10 mm. The raw report also retains the older instantaneous quietness score. The simulation continues past edge loss and through shell contact. Videos replay simulated poses, use a level camera, and label the playback speed.

## Balance with an IMU estimate

We repeated the 10° downhill-right tilt using the same three seeds, 32.2° supported release, head-state feedback, frozen controller and motor model. The instrumented reference reproduced all three original trial records exactly. The new input path feeds measured specific force and angular velocity through the Madgwick IMU update from AHRS 0.4.0 at 50 Hz, as used by the pinned Pollen driver. A static simulated gravity reading initializes tilt before release. For these tests we assumed aligned sensor axes and a site 30 mm above the foot origin. Head pose still comes directly from the simulated linkage. Mounting, sensor bias, oscillator jitter, transport latency and loaded actuator response remain uncalibrated. [Driver used by the pinned SDK](https://pypi.org/project/pollen-bmi088-imu-library/1.0.0rc1/) · [Robot IMU readout](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/backend/robot/backend.py#L535-L572)

| Inputs to the frozen controller | Held edge through return |
|---|---:|
| Original body attitude and interval-averaged gyro | 3/3 |
| Original body attitude and one quantized gyro sample per 20 ms | 0/3 |
| Filter-estimated attitude and one gyro sample per 20 ms, starting at release | 0/3 |
| Same estimator and gyro, table kept level for the matched duration | 0/3 |
| Original inputs until 3 s, then estimator and sampled gyro as the table starts | 0/3 |

With instantaneous IMU samples the body settled inward before the table started. At the handoff the estimated roll was within 0.13° of the actual roll on every seed. All three then lost the original edge 1.46–1.79 s into the excursion and settled flat. No shell landing occurred in these 15 trials. Replacing the gyro input alone was sufficient to break these catches. The point-sampled comparison omits the sensor chip's internal bandwidth.

In a second set of matched trials we modeled the pinned driver's 100 Hz register updates, ±3 g and ±500°/s ranges and 16-bit quantization. Bosch specifies a 40 Hz accelerometer cutoff at the selected normal 100 Hz setting and 12 Hz for gyro register setting 0x05. We approximate each response with a causal first-order low-pass integrated at the physics rate. The controller reads held registers at 50 Hz. We varied the age of each register at readout between 0, 5 and 9 ms. Accelerometer and gyro clocks are phase-locked in this test. The device's detailed transfer functions and the readout phase on the physical unit remain unmeasured. [Bosch BMI088 datasheet, §§4.4 and 5.5.6](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

| Age of reading | Held through the 10° excursion | Landed flat / on shell, moving deck | Landed flat / on shell, level control |
|---:|---:|---:|---:|
| 0 ms | 0/3 | 3 / 0 | 3 / 0 |
| 5 ms | 0/3 | 2 / 1 | 2 / 1 |
| 9 ms | 0/3 | 1 / 2 | 1 / 2 |

The outcome of every paired release was already apparent by the scheduled start of the tilt at 3 s. Each shell contact occurred 1.11–1.28 s after release at about 61° of body tilt. All 33 trials continued to 21.72 s. Timing affected which way the robot fell even on level ground. These three seeds establish a failure of the frozen release controller in this sensor model. The physical unit’s packet ages and best catch timing still need measurement.

The two evaluation commands used 3.66 Wh of whole-Mac energy over 4.98 minutes on the Apple M4 CPU, equivalent to $0.00110 at $0.30/kWh. Together they accrued 26.73 CPU core-minutes. The energy measurement excludes rendering. [Point-sampling receipt](../experiments/reports/compute-meter/moving-table-imu.json) · [Bandwidth receipt](../experiments/reports/compute-meter/moving-table-imu-bandwidth.json)

[Watch three matched releases](../media/simulation/moving-table-imu.mp4) · [Body angle and IMU estimate over time](../media/simulation/moving-table-imu.png) · [Point-sampled trial data](../experiments/reports/moving-table-imu.json) · [Bandwidth and register timing trials](../experiments/reports/moving-table-imu-bandwidth.json)

With Python 3.10–3.12 and the project’s `sim` and `imu` extras, run the following from the project root. The plot additionally uses Matplotlib. The archived Mac run used Python 3.9 with postponed evaluation of annotations in a local copy of AHRS 0.4.0. Its numerical filter code was unchanged.

```sh
python experiments/imu_transfer.py --workers 5
python experiments/imu_transfer.py --bandwidth --workers 6
python experiments/imu_transfer.py --plot
python experiments/film_imu_transfer.py
```

The level-ground refit compares the original controller, a trained feedback policy and command replay on 16 fresh releases. [Follow-up IMU catch experiment](IMU_RIGHT_CATCH.md)

## Controls and compute

All 276 primary qualification trials completed. The waist target stayed at zero throughout. All six starting controls and all 66 duration-matched level controls held their requested edge. At the bracket endpoints, the measured table trajectory met the tracking checks. In the coarse 2°, 256°/s uphill-left test the fixture exceeded the tracking tolerance, so that test cannot establish a precise speed limit. The lower, refined interval uses runs within tolerance.

The existing release controller was frozen throughout. The two metered evaluation intervals on the Apple M4 CPU used **22.23 Wh gross** over 30.10 minutes, or **$0.00667** at an assumed $0.30/kWh. The six-worker processes accrued 133.28 CPU core-minutes. The first interval was interrupted to switch the speed protocol to constant-speed legs. Completed angle trials were reused. Readings cover the entire Mac during those intervals, including concurrent replay or analysis. Additional footage, fine bracketing and footprint analysis outside those intervals have no separate energy total. No GPU training took place. [First meter receipt](../experiments/reports/compute-meter/moving-table-initial.json) · [Resumed evaluation receipt](../experiments/reports/compute-meter/moving-table-qualification.json)

## Replay

From the project root with the pinned upstream CAD assets and the simulation environment installed:

```sh
.venv/bin/python experiments/qualify_moving_table.py --workers 6
.venv/bin/python experiments/moving_table_footprint.py --workers 4
.venv/bin/python experiments/moving_table_feedback.py
.venv/bin/python experiments/film_moving_table.py feedback
.venv/bin/python experiments/film_moving_table.py angle
.venv/bin/python experiments/film_moving_table.py rate
```

The qualification archive contains each seed, table tracking, longest support gap, first sustained loss, shell landing, joint margin, waist target and final state. Rendering uses ffmpeg and a MuJoCo offscreen context. The films have chapter markers and include the failed runs through their final state.
