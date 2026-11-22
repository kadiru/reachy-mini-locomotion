# Catching a tilted Reachy on its CAD foot

**All six previous right-side shell landings now finish on the edge. Four of the six pass the strict final-five-second hold.** From the original stationary 32.2° release the new controller holds continuously for 7.90 seconds of the eight-second trial. The original regulator lands on its lower shell. Holding the neutral head target returns the robot to standing. The original and new left regulators both finish on the edge in all six previous cases, with one strict pass each. These 12 historical cases were available during selection.

On 20 fresh starts between 30.15° and 33.9° with stationary, inward or outward initial motion, the frozen controller recorded **zero shell landings**, compared with seven for the original and eight with a still head target. Ten learned trials ended on the requested edge, including four strict passes. The other ten stood flat. Nine of those retained joint clearance. The left 33.1° flat return passed 0.009 rad beyond a modeled joint travel bound. The original regulator had five edge endpoints and two strict passes across these same twenty starts.

The three additional transfer-sensitivity settings used a 4.95 V bus with 99% nominal PID scale, a 0.55-friction floor, and both changes together. Each ran three releases per side on new noise seeds. Across all 18 trials there were **15 edge endpoints, 10 strict holds, three safe flat returns, zero shell contacts and zero joint warnings**. Each setting used the same three angle/rate pairs, so these counts measure repeatability across those specific perturbations. No physical balance result has been measured.

[Watch the chaptered comparison and every fresh release](../media/simulation/cad-right-release-catch.mp4) · [All trials, training data and power receipts](../experiments/reports/motor-training-cad-release-catch.json) · [Frozen policy](../configs/cad-release-catch.json)

## Every fresh release

Positive rate means falling farther outward. A strict hold counts as an edge endpoint. The film shows the original right controller and both trained sides for each row.

| Starting lean | Starting rate | Original left | Trained left | Original right | Trained right |
|---:|---:|---|---|---|---|
| 30.15° | +1.4°/s | flat with joint warning | flat | flat with joint warning | flat |
| 30.65° | -1.8°/s | flat with joint warning | flat | flat with joint warning | flat |
| 31.2° | +0°/s | flat with joint warning | flat | shell landing | flat |
| 31.7° | +2.2°/s | strict hold | strict hold | flat with joint warning | edge |
| 31.95° | -1.2°/s | strict hold | strict hold | flat with joint warning | edge |
| 32.15° | +1.6°/s | edge | edge | edge | strict hold |
| 32.55° | -2.1°/s | edge | edge | shell landing | strict hold |
| 33.1° | +0°/s | flat with joint warning | flat with joint warning | shell landing | edge |
| 33.65° | +0.9°/s | shell landing | flat | shell landing | flat |
| 33.9° | -1.5°/s | shell landing | edge | shell landing | flat |

## The matched release

We placed the same simulated robot on the right edge of its foot at 32.2° with a neutral head target and zero initial velocity. The original regulator, the trained controller and the neutral-head control begin at identical joint coordinates and velocities. The video shows their actual eight-second trajectories. The slow-motion chapter mirrors the original right regulator onto the left side. The orange dot marks the force-weighted center of pressure in foot coordinates. The torso can occlude that point from the front, so the inset shows the load from above.

At 20 ms the old right controller and its left mirror were both rolling inward at about 6°/s. From 20 to 240 ms both feet changed loaded fore–aft facets, a shift of about 4.9 mm in local x. At 0.5 s measured head roll was +15.7° on the right and −4.4° on the left. The old right controller reversed the inward motion and reached 49.8° of outward lean at 1.5 s, moving at 106.7°/s. Its lower shell touched at 1.572 s and 60.8°. The mirror settled on the left edge. The comparison uses the same seed. The old and new controllers process their sensor noise differently. The mirrored experiment shows that the same feedback law can produce very different head responses on the assembled robot. The model includes asymmetric head geometry and motor loads. These traces alone cannot assign the difference to any one component.

Restricting the original right controller to a neutral pitch and fore–aft target still led to a shell landing at 1.64 s. Limiting commanded roll to ±6° led to a landing at 1.03 s. Applying both restrictions led to a landing at 1.10 s. The old controller continued to receive the full simulated sensor stream. These interventions change its output while leaving its internal command prediction untouched. They illustrate the limitations of clipping an existing controller.

## Learning and sensing

- The waist servo holds zero. The head can roll and shift sideways. The frontal-plane controller holds its other target coordinates neutral. The candidate pool also includes the original regulator and variants with pitch feedback. The first observation is available while the robot is supported.
- For each edge we ran 36 short identification trials with randomized head pulses, a 30–34° release and an initial rate between −3 and +3°/s. A ridge fit estimates next-frame velocities. The ten-state model includes body roll, measured head roll, lateral displacement, their three rates, and the last two two-axis commands. A discrete linear-quadratic regulator computes the gains. The subsequent search runs in MuJoCo on the actual motor-bounded plant.
- Candidate controllers use the simulated body attitude and gyro plus reconstructed head pose. The regulator runs at 50 Hz with a one-frame observation delay after initialization. The simulated gyro is averaged at 1 kHz. The new frontal-plane sensor model applies 0.05° attitude noise and 0.1°/s rate noise. A complementary filter and a fitted head stroke and brake are eligible settings. A one-time selector chooses the controller nearest the observed release tilt plus 0.1 s of measured outward rate. Floor contacts, center of mass and future motion are available for scoring and visualization only.
- The 1.475 kg assembly runs with the pinned shell meshes, MuJoCo convex mesh contacts, 0.7 sliding friction, a hypothetical 5.0 V battery bus, and the project's derated torque–speed envelope. The head joint target speed cap is 10.7 rad/s and the target acceleration cap is 360 rad/s². Physics runs at 8 or 10 kHz. For a moving release we set the initial velocity so the lowest CAD vertex has zero floor-relative velocity. These are assisted initial conditions. Every rollout continues through physical shell landings and has no angle cutoff.
- A 1.8-second neutral-target search on the CAD rim bracketed the switch from inward to outward motion at about **32.20° on the right and 32.37° on the left**. These are simulated release angles on one floor with one motor model. The externally supported robot begins with a neutral target, then gravity deflects its head. Around the tipping point the actual lean after the head settles is approximately 31.44° on the right and 31.59° on the left. At both 8 and 10 kHz, eight-second neutral-target checks confirmed flat returns at 32.182° right and 32.349° left, and shell landings at 32.222° right and 32.389° left. The brackets describe the simulation. A camera frame taken after the head settles measures a different quantity from the initial release angle.
- The strict pass requires the final five seconds on a single loaded edge, outward lean within 2° of 32.1°, absolute roll rate at most 5°/s, pitch within 5°, completion and positive or zero joint margin. The looser edge endpoint uses the final half second with a 3° roll tolerance and a 5° pitch tolerance. A safe flat return requires settling, no shell strike and valid joint travel. A shell landing is recorded on the first positive non-foot floor force. The video reports measured motion at 50 Hz. The collision audit runs at each physics step.

## What the successful right controller learned

The specialist selected near 32° uses a neutral-head equilibrium fitted on the right rim. At that trim the measured head is already rolled about 2.41° and shifted 3.91 mm toward the low side under gravity. The predictor centers on this measured state. Its attitude correction time is 232 ms, its gyro smoothing time is 5.9 ms, and its command smoothing time is 9.1 ms. The fitted control gain is 1.166 and the head-roll target slew limit is 203°/s. The head motor envelope then applies the joint-level limits and delay. The central specialist uses continuous roll and lateral feedback throughout the hold. Other release angles can select a distinct initial stroke and brake.

Four successful metered training commands covered 1,330 MuJoCo rollouts on the Apple M4 CPU with six workers. They took 41.17 elapsed minutes and 201.0 CPU core-minutes, using 30.117 Wh gross at the Mac system sensor. The cash equivalent is $0.00904 at an illustrative $0.30/kWh. The above-background estimate was 7.005 Wh and is sensitive to other activity on the shared Mac. The gross readings include any concurrent diagnostics. No GPU training ran. A separate failed interpreter launch used less than 0.001 Wh. The reported training total excludes that launch, separate qualification and film rendering.

## Reproduction

Install the project's simulation environment and pinned upstream assets as described in the README. From the project root, replay and numerically check the filmed runs against the committed policy and report. The output is written to the ignored experiment directory.

```sh
.venv/bin/python experiments/archive_release_catch.py replay --workers 6
```

All 34 filmed trials reproduced their archived numeric results exactly on the training Mac. Run `.venv/bin/python experiments/archive_release_catch.py verify --workers 4` for the numeric check alone. The report stores the old, new and still-head test outcomes, training conditions, model fit, whole-Mac meter samples and the frozen policy digest. Refit from scratch with the following commands. Each selection uses only development releases. The final paired evaluation runs after the policy is saved.

```sh
.venv/bin/python experiments/train_release_catch.py paired
.venv/bin/python experiments/ablate_release_catch.py
.venv/bin/python experiments/train_release_catch.py screen
.venv/bin/python experiments/fit_release_plane.py
.venv/bin/python experiments/train_release_catch.py plane_screen
.venv/bin/python experiments/train_release_catch.py varied
.venv/bin/python experiments/train_release_catch.py finalists
.venv/bin/python experiments/refine_release_catch.py expand
.venv/bin/python experiments/refine_release_catch.py select
.venv/bin/python experiments/tune_release_hold.py
.venv/bin/python experiments/qualify_release_catch.py validate
.venv/bin/python experiments/qualify_release_catch.py freeze
.venv/bin/python experiments/qualify_release_catch.py test
```

Real foot pads, mass placement, sensor mounting, motor response, bus voltage and floor contact still need measurement. The first physical session uses a supported torso, a fixed camera and read-only daemon sampling. [Supported calibration procedure](SUPPORTED_TILT_CALIBRATION.md)
