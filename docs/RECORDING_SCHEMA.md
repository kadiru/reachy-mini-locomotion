# Recording contract v1

Implemented in `src/reachy_locomotion/recording.py`. JSONL: first line is session metadata; subsequent lines have `kind`, integer `t_ns`, `source`, `data`. Times are nanoseconds since one recorder's `time.monotonic_ns()` origin; lines must be ordered. UTC is for human identification, not duration calculations. NaN/Infinity and malformed transforms are rejected. A new Recorder refuses to overwrite a recording.

Session metadata requires `schema_version: 1`, `kind: session`, `session_id`, `evidence` (physical/simulated/synthetic), `control_mode` (teleoperation/replay/parameter_optimization/learned_policy), `robot`, `setup`, `clock`, and `measurement`. `Recorder` supplies schema/clock fields. The template in `data/session-template.json` is a configuration example, not a recorded trial. Add installed versions, motion hash, trial condition, surface, battery/cabling, attachments, camera calibration and operator information when known. Do not silently promote a synthetic record to physical.

| Kind | Data contract |
|---|---|
| `quest_input` | Raw Emcee payload in degrees, sequence `number`, sender UID, stream ID, optional source clock. The checked-in sender has no acquisition timestamp; capture hook marks it null. |
| `command` | `stage: requested` or `submitted`; `target` has SDK `head` (4×4 homogeneous matrix, translation m), `antennas` ([right,left], rad), and/or `body_yaw` (rad). Include mapping version, associated Quest number and clipping when available. Submission is not an acknowledgment or a measurement. |
| `state` | Preserve complete `sdk_state`; optional request time and `sample_time_basis`. REST matrix poses arrive as `{m: [16 row-major values]}`; they are not silently copied into nested command matrices. Preserve raw daemon timestamp and null IMU. |
| `base_pose` | `frame: world`, `tracked_body: foot`, `x_m`, `y_m`, `yaw_rad`; optional z/roll/pitch, confidence and raw camera PTS. Use only an external base measurement or simulation ground truth, not SDK head FK. |
| `video_frame` | `stream_id`, frame index, video PTS seconds, local media reference, clock mapping ID and residual uncertainty. No video is captured by the present CLI. |
| `event` | `name`; `cycle_start`/`cycle_end` also need unique string `cycle_id`. Other names: `sync_marker`, `fall`, `abort`, `external_assistance`, `state_read_error`, `tracking_lost`. A fall is a new event, not every tilted sample. |

Unknown is `null`/absent, never zero. Base-pose fields are only emitted when valid; omit a lost track and emit a gap event. Raw source packets should be preserved beside any derived values. The validator enforces core fields; open `data` dictionaries allow later hardware-specific measurements.

## Integrating the recovered bridge

Use **one Recorder object** shared by packet callback and state sampler. It serializes writes using a lock. Separate log files have separate origins and cannot be concatenated without explicit clock conversion.

```python
import json
from reachy_locomotion.recording import Recorder
from reachy_locomotion.capture import capture_emcee, poll_state

metadata = json.load(open("data/session-template.json"))
recorder = Recorder("data/physical/trial-01.jsonl", metadata)

# Inside the existing Agora onStreamMessage callback, after decoding:
# capture_emcee(recorder, text, uid, streamId)

# Around the original, recovered SDK call (target mapping deliberately omitted):
# recorder.write("command", {"stage": "requested", "target": target}, "recovered_bridge")
# original_sdk_set_target(**target)
# recorder.write("command", {"stage": "submitted", "target": target}, "recovered_bridge")

# A separate worker may call poll_state(recorder, explicit_robot_origin).
# After stopping and joining that worker: recorder.close()
```

This is integration guidance, not a completed Agora or actuator adapter. Convert numpy matrices/arrays to plain lists before recording. The SDK's built-in start/stop recording API should not be assumed to capture every target, state and video automatically: the audited backend appends records supplied by clients. Our capture scope is explicit.

## Synchronizing video and clocks

1. Keep the external camera fixed, with visible world scale and a rigid foot marker. Record at least a start and end sync flash/marker visible in the video and timestamp its issuance on the recorder host. Characterize display/camera timing uncertainty.
2. Preserve video PTS and fit `t_host_ns = a * pts_seconds * 1e9 + b` from paired markers. Save `a`, `b`, residuals and maximum timing uncertainty. Do not align cameras using wall-clock file creation time.
3. Timestamp Quest acquisition on the sender in a later patch if needed; keep receive timestamps too. Use sequence gaps to detect loss, and a measured sender→host clock mapping before claiming network latency.
4. The present state logger saves request and receipt times plus daemon timestamp. These are not an exact simultaneous sensor exposure. With no clock mapping, report receipt alignment and uncertainty explicitly.
5. Add converted external pose/video records offline, sort by common `t_ns`, validate, and retain untouched raw data. Offline merger/tag tracker are not yet implemented.

Wobbling includes lifting/tilting, so a flat-plane pixel homography alone can misread parallax as displacement. Prefer calibrated 3D tag pose on the foot with known rigid transform and a side view to check contact; validate scale against fixed marks. Synchronization should resolve phase to materially less than a wobble half-cycle. A provisional target is ≤20 ms, to be measured rather than assumed.

## Metrics and their limits

For a cycle with external endpoints `(x0,y0,yaw0)` and `(x1,y1,yaw1)`: `dx=x1-x0`, `dy=y1-y0`, `forward=dx*cos(yaw0)+dy*sin(yaw0)`, `lateral=-dx*sin(yaw0)+dy*cos(yaw0)`, `net=hypot(dx,dy)`. Net endpoint travel is deliberately separate from distance traversed during oscillation. A closed wobble has zero net travel.

Interpolation only uses bracketing poses within `max_pose_gap_s`. No extrapolation or interpolation across a longer internal gap. Duplicate base timestamps, invalid event order and overlapping cycles are rejected. Timing uncertainty and camera calibration must be evaluated separately; the code cannot establish them from a filename.

`xy_endpoint_uncertainty_m` is a user-supplied conservative endpoint position-error bound, not an automatic estimate or standard deviation. The reported displacement bound is twice that value by the triangle inequality; it does not include yaw or clock error. Missing calibration stays null.

All cycles remain in the report, including falls, assistance, aborts, tracking failures and incomplete cycles. Summaries exclude cycles with flagged incidents, but absence of a flag is not proof of no incident. `review_complete` requires both fall and external-assistance review. Unreviewed fall fraction is null. Falls exactly on a shared cycle boundary count in the preceding cycle; session-level events also preserve incidents outside complete cycles. Do not use unreviewed data for a clean-success claim.

Replay-plan preserves submitted target offsets without interpolation or time scaling, rejects command gaps >250 ms or duplicate times, and refuses incident-bearing recordings. This default gap is a data-quality threshold, not a certified hardware watchdog. The output always says `hardware_ready: false`. It does not test kinematic feasibility, send targets, enforce motor limits or simulate motion.
