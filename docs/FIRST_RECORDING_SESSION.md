# First recording session: software and operator guide

Built October 28, 2026. This version implements the Quest bridge, shared-clock recording, 3D camera tracking and stop controls. Verification uses generated camera images and a mock robot. **No real robot, webcam or Quest headset has been tested here; the Unity component is source code, not a compiled APK.** The mapping is reconstructed, with translations locked and configurable rotation gains. It does not recover the unknown mapping used in the GIF.

## What runs where

| Location | Program | Purpose |
|---|---|---|
| Quest Unity scene | `integrations/unity/ReachyRecordingSender.cs` | Read right controller, explicitly report tracking validity, send authenticated packets over the LAN. |
| Mac mini | `record-session` | Receive Quest input, record camera/state/commands, map clutched orientation, show preview and operator controls. |
| Wireless Mini | `guardian --execute` | Sole writer within this application; check profile, IK and rates; enforce an expiring lease; submit to the local Reachy daemon. |
| Mac or robot terminal | `stop` / guardian keyboard | Independent operator stop request. The robot-local timeout still acts if the Mac disconnects. |

The new sender uses direct LAN UDP and does not require Agora or join an existing channel. Keep the old Emcee repository unchanged; copy the component into a working copy of the Unity project. Legacy Emcee packets can still be logged through the existing `capture_emcee` hook, but cannot actuate this bridge because they lack the required tracking-valid/challenge protocol.

Three host modes are explicit:

- `preview`: exercise mapping and recording without robot actuation. The entire session is labeled synthetic, including when a real camera is used for checking the UI.
- `record-only`: record real Quest/camera data and optionally read daemon state. Cannot arm or submit motor targets.
- `execute`: connect to the physical robot-local guardian. Requires matching verified hardware profiles, calibrated live camera tracking and an interactive terminal.

The supervisor is a software control, not a certified emergency stop. Its stop action requests the **current measured head/body pose once** and revokes the lease. It does not snap to neutral, disable torque, wake the robot or automatically resume. Holding a pose does not establish physical stability; that behavior must be commissioned on your robot. A stopped robot must be restored to the measured neutral pose through its normal setup before rearming.

## 1. Install host dependencies and check the software without hardware

On this workspace, the dependencies are already installed in the project-local `.venv`. On another Mac, copy this project and use a separate environment:

```sh
cd /Users/kadir/linkedin/projects/reachy-mini-locomotion
python3 -m venv .venv
.venv/bin/python -m pip install -r experiments/requirements-capture.txt
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion session-smoke \
  --output data/synthetic/my-first-session-smoke
```

Use a new output directory for each run. The smoke test renders known 3D markers, encodes/decodes video, sends real localhost UDP/HTTP messages to a **MockRobot**, checks the recording and removes input to test watchdog stopping. It never imports a real robot adapter or contacts a robot. A passing run is software evidence, not a locomotion result.

Create the private token once:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion make-token data/private/session.token
```

The file is created with owner-only permissions. Copy its value into the Quest component and securely copy the file to Mini when setting up the guardian. Do not commit a scene containing that value. The guardian uses HTTP with a bearer token on your trusted LAN; it is not an internet-facing service.

## 2. Add the Quest sender

1. In a copy of the Emcee Unity project, copy `ReachyRecordingSender.cs` into `Assets/` and attach it to an active GameObject in the scene you build for Quest.
2. In its inspector set `recorderHost` to the Mac's LAN IP, `recorderPort` to `8765`, and `sharedKeyHex` to the token file's contents. Keep the initial send rate at 50 Hz.
3. Use the project's existing XR/Android build configuration and deploy to Quest. Unity/Quest compilation and device behavior remain to be verified on that environment.
4. Stop any old process that sends motor targets to Reachy. The new component does not disable another teleoperator or conversation app.
5. Start the preview below. A valid controller must provide tracking, rotation and button readings. The packet records source acquisition time, sequence and tracking validity. Missing tracking clears all engagement flags.

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion record-session \
  --mode preview --profile configs/first-session-preview.json \
  --token-file data/private/session.token --bind 0.0.0.0 \
  --output data/synthetic/quest-preview-01 --headless
```

Press `a`, release the trigger once, hold **B and trigger together**, then move the controller gently. Its pose at engagement becomes the zero reference. Release either button to stop. Grip also stops; A/Temi engagement conflicts with Mini control and stops the trial. A new `a` and trigger release are required to resume. Body yaw gain is zero in the example; translations and antennas are not commanded.

The example profile has **invented preview limits** and cannot enable physical execution. Preview is for checking flow and signs, not evidence that its gains reproduce the waddle.

## 3. Print and calibrate the camera

Ready-to-print PDFs are in `data/calibration/printables/`:

- `marker-0-80mm.pdf`: fixed world marker on the test surface.
- `marker-1-35mm.pdf`: rigid marker on the lower foot, below the rotating body.
- `checkerboard-9x6-20mm.pdf`: intrinsic calibration board, 9×6 inner corners.

Print at **100% actual size**, then measure the black marker squares and checkerboard square size. Update marker sizes in the calibration JSON if printing differs. Keep the white margins; do not bend the markers. Do not put the foot marker on the moving head/shell. Position the camera so both markers remain visible and each occupies more than 2,500 square pixels—roughly at least a 50-pixel square. A clear oblique view is preferable to a tiny, nearly front-facing marker. The tracker rejects small, clipped, missing, duplicate, high-error or ambiguous tags.

To regenerate the printables, use a new directory:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion make-markers data/calibration/new-printables
```

Take at least ten varied views of the checkerboard, preferably 15–25, covering the image with different positions and tilts. Keep camera resolution/focus fixed afterward. Space saves an original frame; `q` closes the camera. Camera access will use macOS's normal permission flow when you run this locally.

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion calibration-photos \
  --camera 0 --width 1280 --height 720 --output data/calibration/intrinsic-photos
PYTHONPATH=src .venv/bin/python -m reachy_locomotion calibrate-camera \
  --images data/calibration/intrinsic-photos --square-m 0.020 \
  --output data/calibration/intrinsics.json
```

Replace `0.020` with the measured printed square width in meters. Calibration refuses fewer than ten detected views or RMS reprojection error above 1 pixel. A low reprojection error alone does not establish metric accuracy.

Next calibrate the marker-to-foot transform. Keep the world tag fixed on the horizontal surface. Its origin is its center, +X is right, +Y is up on the unrotated print, +Z points away from the surface. Place Mini upright with its foot origin at a measured X/Y position and known heading relative to that print. Measure the lower foot's center/contact-plane reference, not the marker center. Photograph both markers using the same camera and resolution. `calibration-photos` can save this image into a separate folder too.

Example only: a foot origin 15 cm along +X, zero Y, forward along +X:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion calibrate-mount \
  --config data/calibration/intrinsics.json \
  --image data/calibration/mount-photos/frame-000.png \
  --foot-x-m 0.15 --foot-y-m 0 --foot-yaw-deg 0 \
  --output data/calibration/foot-camera.json
```

These values must match your actual placement. The command assumes the foot is upright at world z=0. It derives the rigid marker-to-foot transform; this compensates for the marker moving when the foot rocks.

Run record-only with the camera before enabling any motion:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion record-session \
  --mode record-only --profile configs/first-session-preview.json \
  --token-file data/private/session.token --bind 0.0.0.0 \
  --camera 0 --camera-config data/calibration/foot-camera.json \
  --output data/physical/camera-check-01
```

Record a static interval and, separately, a known ruler-measured translation with motors off. Record any human assistance. Compare the recovered foot positions in `aligned.jsonl` against the ruler; establish drift and a conservative endpoint uncertainty. `c`/`v` can mark an interval for the existing analyzer. Also check that rocking with no net foot travel does not look like forward motion. Record the result in the camera file and set `measurement_verified: true` only after these checks. Intrinsic/mount calibration and this field never certify balance or stop behavior.

## 4. Inspect Mini and commission the hardware profile

This is read-only and does not wake or enable motors:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion inspect-robot \
  --url http://REPLACE_WITH_MINI_IP:8000 \
  --output data/physical/robot-inspection.json
```

Create the parent directory first if needed. This collects daemon status, kinematics engine, motor mode, running moves, current app and joint/IMU state; unsupported endpoints are retained as errors. Preserve the robot's working installation. Do not install the host requirements into its daemon environment or upgrade it during recovery.

Copy `configs/first-session-preview.json` to `configs/first-session-hardware.json` and fill it from the real installation and supervised checks. Required information:

- Robot identity, exact installed `reachy-mini` package version, measured neutral head transform/body yaw and the installed automatic-body-yaw convention.
- Actual seven-joint position bounds, joint speed and tracking-error bounds, task-space rotation/rate/acceleration limits, trial duration, base-tilt limit and workspace radius.
- Confirmed controller axis signs/gains, with translation still locked in this first implementation. Only widen the initial envelope after measured checks.
- Confirmation that conversation gestures, face tracking, other teleop clients and running moves have released the motors.
- Documented measured-pose hold behavior from your hardware commissioning. Record the source of each value and the observation in `verification_notes`.

Set `hardware_verified`, `exclusive_motor_owner_verified` and `stop_hold_verified` only when true. **Do not just flip these flags on the example.** They are commissioning records; the software cannot infer safe motor limits or mechanical stability. Until that information is available, use preview/record-only. The first physical step together is inspecting the installed setup and determining the bounded commissioning procedure; fast wobble execution is not ready merely because unit tests pass.

The adapter currently requires the audited REST field names and `AnalyticalKinematics` engine. It uses the robot's installed analytical IK to check reachability and configured joint bounds before submission. This is not a general mesh collision checker: the chosen task-space envelope must have been physically checked. Motor current/temperature and sensor-level freshness are not established by this REST interface.

## 5. Start the guardian on Mini, then the recording session on the Mac

Copy `src/reachy_locomotion`, the verified hardware profile and the token to a project directory on Mini. Run the guardian **on the robot**, using an interpreter that already has its installed `reachy-mini` and NumPy. Do not point a Mac-hosted guardian at a remote daemon: local daemon access is deliberately restricted to `127.0.0.1`.

On Mini, from that project's directory:

```sh
PYTHONPATH=src python -m reachy_locomotion guardian --execute \
  --profile configs/first-session-hardware.json \
  --token-file data/private/session.token --bind 0.0.0.0 \
  --output data/physical/guardian-01.jsonl
```

Without `--execute`, it is a **mock guardian** and cannot be used by a physical execute session. The guardian has an OS lock against a second guardian for the same local daemon and a per-trial lease. These do not prevent unrelated SDK applications from writing; explicit ownership checks remain required. Unexpected target changes and ignored daemon responses cause a stop.

On the Mac, using the identical profile file:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion record-session \
  --mode execute --profile configs/first-session-hardware.json \
  --token-file data/private/session.token --bind 0.0.0.0 \
  --guardian http://REPLACE_WITH_MINI_IP:8766 \
  --camera 0 --camera-config data/calibration/foot-camera.json \
  --metadata data/session-template.json \
  --output data/physical/teleop-01
```

Fill the metadata template with your surface, cabling, attachments, condition ID and measured camera uncertainty first. Give matched repeated trials the same condition metadata. The program creates a unique session directory and refuses overwrite.

| Control | Effect |
|---|---|
| `a` in terminal or preview window | Request arm; waits for a fresh challenge exchange and trigger release. |
| Hold Quest B + trigger | Clutch current controller orientation, acquire guardian lease and command within profile. |
| Release B or trigger; press Quest grip | Stop and latch. Grip is always a stop, not the old “funny” action. |
| Space, `x`, Escape | Operator stop. |
| `q`, Ctrl+C | Stop and close session. |
| `c` / `v` | Mark cycle/interval start/end. Manual markers need video review for exact phase. |
| `f` / `h` | Record fall / human assistance and stop. |
| `m` | Timestamp an operator marker. It does not automatically create or verify a visible sync flash. |

Keep a second terminal available for:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion stop \
  --guardian http://REPLACE_WITH_MINI_IP:8766 --token-file data/private/session.token
```

The guardian terminal also supports Space/`x` to stop and `q` to exit. A software stop still depends on a responsive robot daemon. A guardian/daemon process crash or power failure cannot be treated as a verified controlled stop; retain physical supervision.

Loss of valid controller input, missing/stale camera pose, excessive configured tilt/travel, impossible/rate-exceeding targets, tracking errors, command rejection and trial expiry latch a stop. Commands cannot refresh the lease using duplicate input or stale preparation. Actual stopping latency includes observation age, deadline, watchdog scheduling and bounded local I/O; measure it on hardware. The example's 200 ms timeout is an experimental setting, not a certified response time.

## 6. What gets saved, and what “synchronized” means here

Each session contains `raw.jsonl`, `aligned.jsonl`, the copied profile/calibration and `camera-original.avi`. The robot keeps a separate guardian JSONL journal with its local command/state times. Original camera frames are saved before UI annotations, with MJPEG compression; frame count is checked by decoding the completed file.

`raw.jsonl` records shared-host monotonic receipt times. Each camera frame has a read interval, receipt timestamp and frame index. Submitted commands carry the guardian's mapped submission time and uncertainty. Authenticated clock exchanges estimate robot-to-Mac offset, bound network uncertainty and reserve a drift allowance. The guardian owns expiration in its own monotonic clock.

`aligned.jsonl` sorts known sample times while retaining raw receipt times. **Camera exposure time and Unity's acquisition clock are not hardware-synchronized.** Source timestamps and this limitation are preserved; no millisecond-level exposure-alignment claim is made. AVI playback FPS is not a substitute for the per-frame timing records. External side-view video still needs visible markers and an explicit clock mapping.

After closing a session:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion validate data/physical/teleop-01/aligned.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion analyze data/physical/teleop-01/aligned.jsonl
```

The analyzer needs interval/cycle markers for displacement summaries. Review the full video and explicitly mark falls/assistance before interpreting “clean” results. Raw logs should stay unchanged; write reviewed metadata/annotations into a new derived file. Recorded zero flags are not proof of no incident. A stop acknowledgment means a hold target was submitted, not that the robot physically settled.

## Evidence and current boundaries

Implemented: authenticated direct-LAN sender protocol, clutched mapping, bounded robot-local guardian, state/command recording, original-frame capture, 3D world/foot tracking, intrinsic/mount calibration, operator/Quest stops, clock-offset logging and receipt-time alignment. Synthetic sender packets are rejected by physical session modes. Preview calibration cannot enable execution.

Not yet verified: Unity compilation/Quest deployment; installed robot compatibility and effective state/target rate; physical controller axes, motor bounds, camera accuracy, stop response, repeatable travel or balance. No real targets were sent during development. The original deployed Emcee mapping remains unknown. Hardware replay, physics calibration, learned policies and speech integration are later work.

Primary references: the locally pinned Reachy SDK's `state.py`, `move.py`, `models.py` and `analytical_kinematics.py`; [OpenCV marker detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html), [camera calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html), and [PnP pose estimation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html). These explain the APIs; hardware behavior still requires observation.
