# Recover the observed wobble locomotion

Updated October 28, 2026, America/Los_Angeles.

## User evidence

The user confirms a **Wireless** Reachy Mini on a **flat surface**, moving by swinging its head quickly so that it wobbles left to right. They have already done this using a Meta Quest 3 and controller. Their description of movement is the starting observation, not dismissed because normal product descriptions call it stationary. It has not yet been independently measured here. Surface material, power cable, pads/attachments, trajectory, speed and fall rate are unknown.

The user supplied [kadiru/Emcee](https://github.com/kadiru/Emcee), and later confirmed they do not remember where the final robot bridge was saved. No further repeated question is needed before reviewing their old machine/robot with them.

Subsequently the user provided `1777717739273.gif`. All 63 frames were inspected: changing controller orientation accompanies head turning/tilting and whole-body rocking; base travel appears toward the laptop in the later segment. One topple, human intervention and a reset/discontinuity are visible. A cable is present; its function/tension are unknown. This is qualitative physical-video evidence, with no calibrated distance or recorded SDK targets. See `GIF_OBSERVATIONS_2026-10-28.md` and the preserved original under `media/user-evidence/2026-10-28-quest-wobble/`.

## Recovered software path

Snapshot: `76ab2fdef8e872d36c0bfd0c866570e512a2aaad`; remote branch inventory exposed only `main` on inspection. This was a shallow snapshot; older history and uncommitted files have not been searched.

| Component | Finding from source | Meaning |
|---|---|---|
| `unityWin/EmceeXR/Assets/AgoraTemiViewer.cs` | Right-hand XR controller, configured `sendRateHz = 50` | This is controller orientation, not necessarily headset orientation. Runtime serialized values may override defaults. |
| Payload | `number`, four engagement booleans, `pitch_deg`, `roll_deg`, `yaw_deg`, `stick_x`, `stick_y` | Preserve raw input and sequence number. There is no source capture timestamp. |
| Pitch | `-(NormalizeAngle(euler.x) + 45)` | This calibration is verified in sender code, not a verified robot-space transform. |
| Roll/yaw | Normalized Unity Euler z/y respectively | Robot axis order/signs, scaling and clipping are still unknown. |
| Buttons | Trigger, A, B, grip → `teleop_engaged`, `a_temi_engaged`, `b_reachy_engaged`, `funny_engaged` | Receiver meaning must be recovered; do not assume B or trigger alone is a sufficient deadman. |
| Transport | Agora data stream, `SendStreamMessage` | Recover deployed SDK/runtime before rebuilding. No channel was joined here. |
| `pythonReachy/reachy_teleop_rtc.py` | `PrintOnlyEventHandler.onStreamMessage` decodes and prints | This snapshot contains no call into Reachy SDK, no target mapping and no actuator loop. |
| Lost controller | Sender zeros pose/stick but retains previous button states | Deployed bridge must explicitly drop engagement on tracking loss/timeout; retained button flags cannot be trusted as a deadman. |

Source links: [sender](https://github.com/kadiru/Emcee/blob/76ab2fdef8e872d36c0bfd0c866570e512a2aaad/unityWin/EmceeXR/Assets/AgoraTemiViewer.cs), [receiver](https://github.com/kadiru/Emcee/blob/76ab2fdef8e872d36c0bfd0c866570e512a2aaad/pythonReachy/reachy_teleop_rtc.py).

Capture hooks now accept these packets without guessing their motor mapping. At the original receiver, record raw packet reception, requested SDK target immediately before the call, and target submission immediately after a successful call. SDK submission does not prove physical execution; joint state supplies that separate evidence.

## Where to look next

On the machine/robot that ran the original demo, inspect project filenames and process/service names for `reachy`, `teleop`, `agora`, `Emcee`; then search only likely source directories for `ReachyMini`, `set_target`, `create_head_pose`, `pitch_deg`, `b_reachy_engaged`. Preserve files and record installed package versions and startup command before changing anything. No remote host or credentials are known here, and no broad home-directory search or remote connection was attempted.

If the bridge cannot be recovered, rebuild a *new* mapping at low amplitude, log its configuration and label it reconstructed. Do not call it an exact replay of the previous approach. First check axes one at a time, with a rest baseline and external side view. Identify whether roll, lateral translation, pitch, body yaw or a combination caused unloading and slip.

Independent fallback reference: [HRI-EU/reachy-vr-quest](https://github.com/HRI-EU/reachy-vr-quest) sends head/body/antenna targets through the daemon's WebSocket, with a separate video connection. It is a useful alternative, not evidence that the user used that bridge. Its default mock tracking provider also needs replacing for real Quest input.

## What motor and balance information is available

Audited official SDK commit: `a9a2a42adcc1abc7d9c90d390889bbebf930c96c`. Availability on the user's older installed version remains unverified.

| Signal | Access and interpretation |
|---|---|
| Measured joint positions | `/api/state/full?with_head_joints=true` returns body yaw then six Stewart joint angles; antennas are separate. Units radians. |
| Requested joint/pose targets | Opt-in `with_target_head_joints`, `with_target_head_pose`, `with_target_body_yaw`, `with_target_antenna_positions`. Sampled targets may miss intermediate commands; tap the bridge too. |
| Head pose | Kinematic transform estimated from encoders. Its SDK reference frame assumes the base reference; it is not external base odometry. |
| Wireless IMU | `with_imu=true`, `/api/state/imu`, or SDK `mini.imu`: acceleration (m/s²), angular rate (rad/s), quaternion (w,x,y,z), **IMU** temperature (°C). Null can mean unavailable/stale. |
| Joint speed | Can be estimated from encoder differences using actual timestamps and filtering; label derived, not a measured speed register. |
| Tracking error | Compare requested and measured joint/pose states after estimating transport/controller lag. |
| Motor current, load, motor temperature | Not exposed in the inspected standard full-state API. Do not interpret IMU temperature as motor temperature. Hardware-specific daemon instrumentation may be possible after motor/firmware identification. Do not add a competing serial-bus reader. |
| Balance/contact | IMU and video give tilt/rotation/contact timing clues; contact forces, center of pressure and support margin are not directly measured. Calibrate sensor-to-foot transform before treating IMU orientation as base tilt. |
| World displacement | External calibrated camera/tag on the foot or motion capture. Never integrate head pose, body-yaw motor angle, or IMU acceleration as if they directly measured base travel. |

The current backend updates the IMU inside its control loop and serves a cached sample; polling more quickly does not guarantee fresher measurements. Host receipt timestamps include network delay. Some integration prose in the repository is stale about IMU route availability; the inspected route and backend code above are the basis for this table.

Technical sources: [state route](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/app/routers/state.py), [state model](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/app/models.py), [robot backend](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/backend/robot/backend.py), [SDK IMU guide](https://huggingface.co/docs/reachy_mini/en/SDK/python-sdk).
