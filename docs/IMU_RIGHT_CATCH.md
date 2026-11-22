# Catching a supported right-edge release with a 50 Hz IMU

The question is whether Reachy can catch a slight variation in its starting pose using the IMU available to the simulated controller. Every test starts with a supported tilt on a level deck. The robot has a free base, CAD shell contacts, a zero waist target and the existing 5 V battery motor model. It runs for eight seconds through any inward or outward landing. The baseline is the prior right-edge controller receiving the same modeled sensor packets.

## Results

| Controller | Held at 32.2° stationary | Held with a changed starting angle and rate | Settled on the foot | Landed on the shell |
|---|---:|---:|---:|---:|
| Original with 50 Hz IMU | 0/8 | 0/8 | 13 | 3 |
| Refit with IMU and head feedback | 8/8 | 0/8 | 7 | 1 |
| Same head commands replayed with corrections disabled | 8/8 | 0/8 | 5 | 3 |

The eight nominal holds followed the same physical trajectory. Across all five sensor ages the trained controller made zero corrections above the measurement threshold of one millionth of a degree or millimeter. Its recorded simulation poses matched command replay exactly, and it finished at 31.4° body roll. Changing the starting tilt by only 0.15–0.25° while adding 0.4–0.8°/s of initial rotation broke the hold in every test. These cases vary angle and rate together.

Feedback changed the landing. From 32.45° with −0.8°/s initial rotation, replay struck the shell at 0.804 s. The refit settled onto the foot at all three sensor ages. From 31.95° with +0.8°/s and a 9 ms reading age, the refit hit its shell at 1.668 s while replay settled onto the foot. Feedback intervened on 399–401 of 401 samples in the changed releases. All 48 trials finished at the fixed waist target. [Watch every matched release](../media/simulation/imu-right-catch.mp4) · [Per-trial contacts, correction counts and training results](../experiments/reports/imu-right-catch.json) · [Frozen parameters](../configs/imu-right-catch.json)

The next simulation experiment should separate starting angle from starting speed. Use symmetric stationary angle offsets around 32.2°, then small inward and outward rates at exactly 32.2°. Compare each refit with replay and reserve new mechanical starts. A successful correction should hold an edge where replay loses it. For physical calibration, measure tilt, head motion and IMU timing while the torso is supported. [Supported physical calibration](SUPPORTED_TILT_CALIBRATION.md)

## What the controllers can sense

The modeled BMI088 filters acceleration and angular velocity, publishes registers at 100 Hz and supplies data to the controller at 50 Hz. A Madgwick estimate initialized during the supported setup supplies tilt. The physical mounting, calibration and packet timing remain unmeasured. Policies with head feedback also receive the simulated linkage pose. Contact measurements, true torso pose and future sensor packets are available only to the scorer or to the offline teacher. The controller has no input for sensor age, seed or release angle other than what it observes during the episode. [Sensor model and original comparison](MOVING_PLATFORM_BALANCE.md#balance-with-an-imu-estimate)

## Training and independent controls

1. The first search screened 36 adjustments to the previous 32.2° regulator, including feedback gain, damping, command slew and filtering. Eight candidates advanced to three timing conditions and three advanced to eight-second tests.
2. A second fit used nine simulated demonstrations to learn the difference between the sampled roll gyro and the teacher’s interval-averaged roll rate. A regularized linear predictor sees four consecutive accelerometer and gyro packets plus three intervals of head roll and lateral displacement. Six combinations of observer and regulator were tried. The successful four-second examples did not last eight seconds on the corresponding training seeds.
3. A successful eight-second demonstration supplied a head trajectory and a range of expected IMU readings at each instant. The same teacher motion was observed at register ages of 0, 5 and 9 ms. Twelve feedback settings were tested on three releases, then three settings and the prior best were tested on five releases. Errors outside the observed range trigger a correction to head roll and lateral displacement. There are separate bounds for head motion and motor torque, speed and travel in the plant.
4. The final fit evaluated 12 alternatives that also use measured head roll, lateral position, their changes and previous head commands to predict a correction. Four finalists faced the same five training releases. The winning configuration was frozen before evaluation. This eight-second trajectory provides a finite-duration reference. Sustained regulation past its end has not been tested.

The fresh suite includes 16 cases. Two new sensor-noise seeds repeat the nominal 32.2° stationary release at 0, 5 and 9 ms, while separate starts at 31.95° and 32.45° have initial angular velocities of +0.8°/s and −0.8°/s. Two further sensor ages, 2.5 and 7.5 ms, are each tested at 32.2° stationary and 32.05° with +0.4°/s initial motion. The timestamps are constant within each episode. A command-replay control follows the same head trajectory with corrections disabled. The noise seeds perturb controller observations. Repeating the same mechanical start and head commands gives the same deterministic plant motion.

A pass requires the requested right edge at every 50 Hz observation over the final two seconds, no support gap lasting 20 ms in the independent 1 kHz contact audit, and a steady final second. The last second permits 3° ranges in roll and pitch, 5° in yaw and 10 mm foot-center travel. The complete eight seconds must finish with positive or zero modeled joint margin and no shell contact. A full landing and a flat-foot return are reported separately. [Original contact and steadiness definitions](MOVING_PLATFORM_BALANCE.md#what-counts-as-holding-the-edge)

## Reproduce the experiment

The four fitting stages comprised 211 simulated episodes, including the teacher demonstrations. With six CPU workers on an Apple M4 they used 5.280 Wh of whole-Mac energy in 7.64 wall minutes and 37.34 CPU core-minutes. At $0.30/kWh the gross fitting cost was $0.00158. Mean system power by stage was 41–42 W. The GPU was idle for training compute. [Gain search receipt](../experiments/reports/compute-meter/imu-catch-fit.json) · [Rate estimator](../experiments/reports/compute-meter/imu-catch-observer.json) · [Head trajectory](../experiments/reports/compute-meter/imu-catch-reference.json) · [Head-state feedback](../experiments/reports/compute-meter/imu-catch-linked.json)

The completed 48-run evaluation used 1.566 Wh over 2.09 minutes. An interrupted evaluation used another 0.962 Wh during a low-storage interval. We restarted the complete frozen suite and checked the saved pose arrays against replay for all eight holds. The metered total including both evaluations and training was 7.807 Wh or $0.00234. The individual diagnostic replay, regression tests and rendering were unmetered. [Completed evaluation](../experiments/reports/compute-meter/imu-catch-qualification.json) · [Interrupted evaluation](../experiments/reports/compute-meter/imu-catch-evaluation-interrupted.json)

Run from the project directory with the simulation dependencies, the pinned CAD assets and the IMU extra installed. The qualification stage saves each 50 Hz replay locally, checks the policy hashes, and writes the report. The video uses those same poses. The archived Mac environment used the Python 3.9 annotation compatibility adjustment documented in the sensor study.

```sh
python experiments/train_imu_catch.py fit --workers 6
python experiments/train_imu_catch.py observer --workers 6
python experiments/train_imu_catch.py reference --workers 6
python experiments/train_imu_catch.py linked --workers 6
python experiments/train_imu_catch.py qualify --workers 6
python experiments/film_imu_catch.py
```
