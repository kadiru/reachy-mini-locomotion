# Reachy Mini: from your Quest wobble to “Mini, walk forward”

October 28, 2026 · America/Los_Angeles

This is the implementation and hardware runbook. Proposed programs, trial counts, thresholds and training settings below are a design, not completed experiments. No policy has been trained or deployed. The immediate goal is a short, supervised forward waddle on one measured surface, followed by a controlled stop.

**Implementation update:** the first-session bridge, integrated recorder, camera calibration/tracking and robot-local guardian have now been written and checked with synthetic inputs/mock hardware. See `FIRST_RECORDING_SESSION.md` for current commands and verification. Later-program descriptions below remain the training design; statements about missing capture programs describe the earlier planning baseline. Real hardware and the Unity build remain unverified.

**The route:** record your successful motion → replay it → build and validate its physics model → learn small feedback corrections → expose the resulting walking skill to your existing conversation system.

You supply the physical setup, demonstrations, resets and supervised trials. The software work supplies recording, tracking, replay, simulation, training, evaluation and speech integration. You do not need to manually label thousands of frames or train a new language model.

## 1. What we know, and what we still need

- Your robot is Wireless, and your GIF supports controller-driven head turning/tilting and whole-body rocking. It also contains a topple and a reset. It cannot give us calibrated speed, the exact motor commands or the original timing.
- Emcee contains the Quest sender. Its checked-in Python receiver only prints packets. The deployed robot mapping remains unknown. We will inspect the old robot/computer when available; if it cannot be recovered, build and label a reconstructed mapping.
- Recording/validation, a read-only state logger, displacement analysis and an offline replay-plan generator already exist in `src/reachy_locomotion/`. Their 14 tests passed previously. Hardware actuation, camera tracking, a moving-base environment, training and speech integration remain to be written.
- The audited stock MuJoCo scene has a fixed foot, no foot contact geometry and idealized actuators. It is not ready for this locomotion task. See `SIM_AUDIT.md` and its compiled report.
- You report an earlier Mac mini/Ubuntu local-SLM setup for Temi and Reachy. Its exact code, endpoints and installed models have not been recovered. The local `woz-demo-stack-main` README describes a GO2/SO-101 project; the inspected files did not establish that it is this speech system. No services were started or modified.

## 2. Put these pieces of hardware together

| Hardware | Your preparation | Intended job |
|---|---|---|
| Reachy Mini Wireless | Charge it; preserve the existing working installation; photograph the underside/pads and attachments. | Execute joint targets; supply available joint and IMU state. |
| Quest 3 + right controller | Charge them; launch the original Emcee build if available. | Demonstrate the motion. They are unnecessary during autonomous execution. |
| Mac mini | Put it on the same trusted LAN as the robot; have its normal display/keyboard available. | Initial operator console, recording, external camera processing and speech/skill routing. This placement is a proposal until the old setup is recovered. |
| Ubuntu computer | Preserve the existing local SLM and humanoid jobs. | Keep serving the existing language model; later run isolated CPU simulation/training if resources are available. Training can instead start on the Mac. |
| Fixed external camera | Use an existing USB webcam for live tracking if possible; an existing phone can record an additional side view. Use original video, preferably 60 fps; verify actual timestamps and blur. | Measure where the lower foot moves. Offline phone video helps analysis but does not automatically provide live policy feedback. |
| Printed marker + ruler | Attach a small rigid marker to the lower foot, below the rotating body joint, where the camera sees it. Place a measured scale and fixed world markers in view. | Separate true base translation/rotation from head movement and camera motion. |
| Flat, firm test surface | Use a floor-level board with the original tabletop material if available; leave a clear area. Mark a start position and forward arrow on the surface, without changing the underside contact. | Repeatable contact physics and room to stop. |
| Temi | Keep it parked for initial locomotion tests. | Conversation integration after Mini can execute and stop reliably. |

Use battery operation for an untethered test if feasible. The GIF has a cable: document what it did. If the original successful motion used it, record cable-present and battery-only conditions separately. A tether can change both stability and travel. Do not silently add grippy feet, weights or a supporting harness and call it the same mechanism.

The observed fall makes a floor-level test area useful. Keep a hand ready to catch the robot; mark every catch/contact as an assisted trial. Do not deliberately tip the powered robot to collect failures. Set it upright by hand after a fall; self-righting is a separate future task.

## 3. First hardware session: recover control and capture one useful trial

Perform these in order. Progress depends on evidence, not a promised number of hours.

1. **Inventory before changing software.** Record robot identity, daemon/SDK versions, IP/hostname, camera identity, battery/cable state, surface and attachments. On the old Mac/Ubuntu/robot, inspect likely service names and source directories for Emcee, Agora and Reachy. Preserve the deployed bridge and its configuration. Do not upgrade a working installation during recovery.
2. **Read state at rest.** Use the existing read-only logger for a 10-second availability check. Then capture a 20-second static camera/state baseline. Determine which fields are populated, their actual update rate, timestamp meaning and noise. A rapidly polled cached value is not a fresh measurement.
3. **Establish one motor owner.** Suspend other head/body command producers during calibration and walking: idle animations, face tracking, speech gestures and other teleoperators. Audio can remain available. A mode selector chooses TELEOP, REPLAY or POLICY; it never runs two writers together.
4. **Verify neutral, signs and stopping.** With a reconstructed bridge, begin with small individual axis changes within the installed SDK limits. Verify neutral controller pose, degrees-to-radians conversion, coordinate frames, engagement and release. Recover the coupled turning/tilting motion only after these checks. Verify controller loss, stale commands and local stop behavior before a fast swing.
5. **Calibrate the external camera.** Estimate intrinsics and world orientation/scale, plus the rigid marker-to-foot transform. A moving/tilting marker needs 3D pose estimation; a flat image-to-table mapping can mistake rocking for travel. Check stationary drift and a known measured displacement. Save calibration and uncertainty.
6. **Record one short demonstration.** Start with rest, do a brief familiar wobble, then return to rest. Record the complete attempt, including failed starts. Preserve raw Quest packets, submitted SDK targets, actual joints, available IMU, camera pose and original video.
7. **Check the recording immediately.** Confirm the foot moved beyond static drift and measurement uncertainty. Check timestamps, target/state lag, tracking gaps and human contact. Only then collect more examples.

The present read-only command, after copying and filling out the metadata template with a unique trial ID and the real robot address, is:

```sh
cd /Users/kadir/linkedin/projects/reachy-mini-locomotion
PYTHONPATH=src .venv/bin/python -m reachy_locomotion capture-state \
  --url http://REPLACE_WITH_ROBOT_IP:8000 \
  --metadata data/session-template.json \
  --output data/physical/first-state-check.jsonl --seconds 10 --hz 20
```

It does not enable motors or collect video/Quest targets. The other launch commands will be provided when those programs exist and the installed robot API is verified.

**Data to gather next:** aim initially for ten short attempts of roughly 5–8 seconds of motion, with rest before/after and a reset between attempts. This is a starting collection budget, not an amount known to be sufficient for learning. Stop earlier on repeated falls, abnormal operation or unresolved recording faults. Keep all attempts; successful segments become candidate demonstrations, and failed/assisted segments remain failure evidence.

Add separate gentle axis-response trials to estimate lag and tracking, plus stationary controls. Avoid testing maximum motor strength, stall torque or the tipping boundary. We need useful dynamics within the demonstrated operating range.

## 4. What the recorder and measurements must contain

| Stream | Required information | Purpose |
|---|---|---|
| Session | Hardware/software versions, bridge hash, surface, battery/cable, start pose, camera calibration, mode | Reproduce conditions and distinguish physical/simulated/synthetic data. |
| Quest | Orientation, buttons, sequence, acquisition time if available and receive time, tracking validity | Recover the demonstration and detect stale/lost input. |
| Robot commands | Requested and actually submitted head transform/body yaw, timestamps, mapping and clipping | Train/replay what was sent, rather than guessing from hand motion. |
| Robot state | Measured joints, targets, IMU if available, source/receipt times | Measure response and feedback latency. |
| External foot pose | Position, orientation, confidence, video timestamp | Ground-truth displacement and the first version's live motion feedback. |
| Events | Start, cycle boundaries, stop request/completion, gaps, faults, falls, catches, resets | Honest denominators and stopping analysis. |

Use one recorder clock where possible. Preserve raw timestamps and fit mappings for external clocks using start/end sync markers; log the alignment error. The current JSONL tools support these event types, but a synchronized camera/Quest/robot capture application still needs integration. See `RECORDING_SCHEMA.md`.

The inspected SDK exposes joint states and a Wireless IMU. Head forward kinematics does not measure world travel. IMU acceleration cannot simply be integrated into reliable centimeter-scale odometry. Validate IMU mounting/frame conventions against the external camera; if it moves with a joint, compensate with the measured kinematics. Use external tilt until a base-tilt estimate is validated. The standard state interface inspected here does not expose motor current, motor temperature or contact forces. IMU temperature is not motor temperature. Keep unknown quantities unknown; instrument further only through the existing daemon/bus owner.

## 5. Turn the demonstration into a repeatable baseline

First write a supervised replay executor around the existing offline replay plan. Preserve the original submitted target timing. It must check the starting pose, transition into/out of the motion, enforce the calibrated envelope and respect the mode/stop controller.

Replay one clean complete cycle, then three, then the proposed five independent trials of ten cycles if the shorter tests succeed. Reset start location and heading between trials; keep the same surface, battery condition and camera setup. Include stationary controls and, once stable, a second starting heading to detect a slope or cable bias. Shortening a trial is different from slowing its swings: slower motion changes the inertial mechanism.

Report signed forward travel, lateral drift, heading change, completed/attempted cycles, falls, assistance, tracking loss, tracking error and time to stop. Every attempt belongs in the denominator. Compare methods under matched conditions; the current CLI `compare` handles same-mode repeated trials, so a cross-method evaluation report still needs implementing.

Next fit a compact phase trajectory to the clean motion. Retain recorded translations and coupled rotations, including any body-yaw component. Fit amplitudes, offsets, relative phases and period; use a periodic spline or a few harmonics only if they reconstruct the recorded targets well. Keep a separate start and stop segment. Smooth the cycle seam and validate it before repeated execution.

A bounded parameter search in simulation can improve this baseline before RL. It is also the baseline that the learned policy must beat. If replay already satisfies the voice demo, it can be exposed to speech early and labeled **voice-triggered replay**. Learned feedback remains a separate milestone.

## 6. Build the simulation that can actually learn this behavior

Create a separate experimental MuJoCo scene from the pinned official model; preserve the reference copy.

1. Add a free-moving foot/root and explicit collision shapes matching the underside/rim/pads. Preserve the Stewart platform's closure constraints and passive joints.
2. Keep robot-relative SDK targets consistent when the root moves. Use the measured world-to-foot transform to compute the head pose in the proper robot reference frame; do not reuse the stock constant-height subtraction as moving-base kinematics.
3. Weigh the assembled robot if a suitable scale is available; measure the support footprint and photograph contacts. Start from official inertias, but record discrepancies and uncertainty. Total mass alone does not identify center of mass or inertia.
4. Model the installed actuator response: delay, position tracking, damping, rate/saturation limits and observed deadband. Establish limits from the actual hardware documentation/configuration and gentle response data. The audited simulation's ±20 force range is not a real motor rating.
5. Fit contact/friction and actuator parameters to the physical recordings. Match joint response, rocking amplitude/phase, contact sequence, travel direction and displacement. Several parameter sets may fit; keep an uncertainty range rather than claiming a unique friction/COM estimate from one clip.
6. Reserve entire physical trials for validation. A useful initial split for ten attempts is six fitting, two validation and two untouched tests; collect more when clean coverage is insufficient. Never split neighboring frames across train/test and call them independent.
7. Replay held-out targets. The stationary model should not drift, and observed travel direction and fall-free behavior should agree. Predeclare tolerances using tracking error and between-trial variability. A model whose discrepancy is larger than the candidate improvement cannot support a credible ranking of policies.
8. Check that travel persists when changing solver settings and halving the physics timestep. Reject numerical vibration, unrealistic force or penetrating-contact solutions. Model reset must settle the real support geometry without injecting a propulsive impulse.

Start from the audited 2 ms physics step and a candidate 50 Hz control update only if hardware timing supports it. These are separate rates. Contact fidelity may require a smaller physics step. Domain randomization comes after fitting: vary measured/plausible friction, contact geometry uncertainty, inertia, initial tilt, actuator lag, sensor noise and network delay. Keep broad unmeasured ranges labeled exploratory.

MuJoCo's [modeling documentation](https://mujoco.readthedocs.io/en/stable/modeling.html) explains movable bodies and contact parameters. Our specific fixed-foot findings and required changes come from the local audit, not from a completed locomotion simulation.

## 7. Define exactly what the learned policy does

**Recommended first design: a small residual policy.** A verified periodic trajectory supplies the main wobble. The policy reads feedback and adds bounded corrections. All final targets pass through the same geometric feasibility, joint, speed and supervision checks used in replay.

Initial action space: four normalized residuals in `[-1, 1]`: head rotation about three calibrated robot axes and body yaw. Preserve any translations in the demonstrated baseline; add translation residuals only if data shows they are needed. Compose head rotations consistently as transforms, rather than mixing Unity Euler angles with SDK axes. Never issue six arbitrary Stewart motor commands without satisfying its kinematics.

An initial simulation residual scale can be 5% of each demonstrated active axis's peak-to-peak range. This is a proposed search setting, not a verified safe physical limit. An axis with no demonstrated motion stays disabled. Combined baseline plus correction must remain in the validated feasible envelope. Increase only after evaluation. Keep antennas in a fixed documented pose for initial experiments.

Policy observations, all available at deployment:

- Seven measured body/Stewart joint angles and filtered estimated velocities.
- Validated base tilt/gravity direction and angular velocity; use the external tracker when IMU transformation is unresolved.
- Previous residual action, baseline phase as sine/cosine, and current motion mode.
- External foot forward/lateral velocity, lateral error and heading error relative to the command's initial frame.
- Remaining target distance, when using the distance-controlled skill, and valid sensor age information.

Derive velocities with the same causal filter in simulation and hardware. Use only past samples; reproduce filtering delay. Reject stale observations outside the policy. Normalize using training statistics and freeze them for evaluation/deployment. Start with four observation frames if one frame is insufficient; preserve identical history initialization at reset. Do not feed simulator-only contact forces or perfect world state to the deployed actor.

**Camera dependency:** this first design needs a calibrated live external camera at runtime. Its footprint is the tested workspace, not arbitrary rooms. An eventual version without that camera needs a separately validated onboard state estimator or a proprioceptive policy, and a new evaluation. The existing head camera moves during the gait and cannot be assumed to supply equivalent odometry. A timed burst can work without metric odometry, but cannot truthfully promise a distance in centimeters.

## 8. Training tasks and success definitions

Use separate stages, retaining earlier tasks during later training so the robot does not forget to settle.

| Stage | Episode/task | What counts as progress |
|---|---|---|
| A: Rest | Begin near the measured rest pose; hold for a short interval. | Little base drift, stable joint response, no sustained saturation. Establish a baseline before learning. |
| B: One cycle | Execute one demonstrated cycle with residuals, then settle. | Positive signed progress beyond measurement-equivalent noise; no fall or contact anomaly. |
| C: Forward burst | Start, perform several cycles, stop and settle. | Net forward progress with bounded lateral/heading error and no assistance. |
| D: Reliable stopping | Interrupt at varied cycle phases and target distances. | Reaches the verified stop state within the measured envelope, without an abrupt destabilizing reset. |
| E: Variation | Repeat B–D with held-out starts and fitted parameter/noise/delay variations. | Maintains performance across conditions, rather than only one simulator seed. |
| F: Physical transfer | One cycle → three cycles → finite target-distance trials. | Measured physical performance matches the acceptance criteria below. |

A provisional demonstration target is **5 cm forward within 20 seconds, lateral error at most 2 cm, final heading error at most 15°, followed by two seconds settled**. These are proposed goals, not inferred capabilities. After measuring the baseline, choose and freeze achievable numbers before comparing methods; record any revision. Define “settled” quantitatively from the static pose/velocity noise and verified joint tracking. Require the target to be maintained through settling, rather than rewarding a transient crossing during a swing.

Initial physical acceptance: at least 9 of 10 independent attempts complete the frozen target, with zero falls, zero catches and no missed stops in that batch. Log the tenth attempt even if it fails. This is a demo acceptance gate, not proof of a low long-term failure rate. Expand to another day and surface only after passing; report each surface separately. Backward motion, turning, navigating around furniture and self-righting are additional tasks, not consequences of learning forward motion.

**Starting reward design, to tune on training data:**

Let `L` be target distance, `T` the episode time scale, `Y` the lateral tolerance, `Psi` the heading tolerance; all are positive, fixed for an experiment. Let `s` be signed forward position from the start. One possible dimensionless reward is:

```text
r = 2 * (s_next - s) / L
    - (dt / T) * [0.5 * (lateral_error / Y)^2
                  + 0.25 * (heading_error / Psi)^2
                  + 0.25 * excess_tilt^2
                  + 0.05 * mean((action - previous_action)^2)
                  + 0.25 * normalized_joint_tracking_error^2]
    - 5 * failure_event
    + 1 * completed_and_settled_event
```

All coefficients are starting proposals. Normalize tilt excess above the demonstrated phase-dependent rocking envelope; do not penalize the essential left/right rock as if the foot must always stay flat. Use a separate stop/rest objective penalizing residual base speed and distance error. Clamp/terminate out-of-bounds states before rewards can be exploited. Keep success and failure bonuses one-time. Signed progress cancels backtracking; do not reward absolute distance traveled or head movement. Smoothness is a command penalty, not measured energy. Motor-energy claims require validated torque/current measurements or an explicitly labeled model estimate.

End an episode on a fall, assistance, boundary exit, invalid observation, impossible target or supervisory violation. Preserve its failure reason. Handle an external rollout time limit as truncation; a task deadline is a task failure if that deadline is part of the task, with remaining time included in the observation. Record timeout failures in evaluation regardless of the API flag. Implement the [Gymnasium environment API](https://gymnasium.farama.org/introduction/create_custom_env/) with explicit `reset`, observation/action spaces and `step` outputs.

## 9. Training run and evaluation procedure

First compare raw replay and the fitted trajectory in the validated simulator. Optionally use behavior cloning to initialize a feedback model from demonstrations, with whole-trial splits. A phase-only fit learns a motion pattern; feedback robustness still needs evidence.

For RL, a reasonable initial experiment is PPO with a small MLP, using CPU resources:

| Setting | Proposed starting value |
|---|---|
| Networks | Separate actor/value MLPs, two hidden layers of 128 units |
| Device/workers | CPU; begin with one environment for debugging, then four if resources permit |
| Control update | Candidate 50 Hz, adjusted to measured end-to-end timing |
| Rollout | 1,024 steps per environment |
| Batch/epochs | 256 / 10 |
| Learning rate | 0.0003 |
| Discount/GAE | 0.99 / 0.95, reconsider when changing control frequency |
| PPO clipping | 0.2 |
| Seeds | Three independent training seeds; fixed, disjoint evaluation seeds |
| First budget | 10,000 steps to check behavior and accounting; then 100,000 as a learning diagnostic |
| Extension | Consider 1 million steps only after diagnostics show useful learning; review before extending further |

These step counts and hyperparameters are experimental starting points, not a convergence promise. No job is launched by this document. Use a fresh project environment compatible with the selected libraries; preserve the current Python 3.9 audit environment. The [SB3 PPO documentation](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html) supports continuous actions and CPU execution for small MLP policies. Additional GPUs are unnecessary for the first experiment and existing humanoid GPU work retains priority.

At each checkpoint, evaluate fixed held-out scenarios without exploration noise, compare against replay, and inspect rendered failures. Track success/falls, lateral/heading error, stopping time/distance, tracking error, action saturation and learning curves. A reward increase without better task metrics is not promotion. Select using validation scenarios; run the untouched final test suite only after selection. Keep model/config/environment/data hashes and normalization alongside each checkpoint.

Before hardware, test the exported runtime against the training implementation on recorded observations. Confirm identical preprocessing, observation ordering, action scaling and near-identical outputs. Benchmark inference latency on the chosen runtime machine. Test watchdogs with simulated stale streams, dropped commands, duplicate requests and process failure.

On hardware deploy a frozen deterministic policy; weights do not update during the first trials. Begin with one cycle and the same conditions as replay. Preserve all failures and compare with interleaved replay controls. If transfer is poor, inspect mismatched contact, lag, state estimation and saturation; do not compensate by granting larger motor commands. Physical fine-tuning is a later supervised design, not unrestricted on-robot PPO exploration.

## 10. Programs and artifacts to implement

The names in this table are proposed modules under `src/reachy_locomotion/` unless marked existing. Several can share one application/CLI; they do not need sixteen separate servers.

| Module/artifact | Responsibility and completion check |
|---|---|
| `robot_io.py` + hardware profile | Versioned daemon adapter, read state, submit SDK targets through one owner, measured timing, installed limits and frame conventions. Dry-run contract tests precede an observed small-axis check. |
| `supervisor.py` | Mode/ownership, feasible envelope, rate limits, stale sensing/commands, stop and fault state machine. A local guardian must expire commands even if the Mac/network fails. Verify actual daemon behavior; if no suitable timeout exists, implement the guardian on the robot before walking. |
| `quest_bridge.py` | Recover or reconstruct Emcee-to-SDK mapping; add acquisition timestamp and explicit tracking-valid signal to sender; log input and targets. Lost controller must revoke engagement despite retained button flags. |
| Existing `recording.py`, `capture.py` + `capture_session.py` | Integrate command/state/camera workers on mapped clocks, preserve originals, write session/event metadata and counts. Read-only capture already exists; synchronized capture does not. |
| `track_base.py` + calibration files | Intrinsic calibration, 3D marker pose, world/foot transform, causal velocity, uncertainty/quality and gaps. Pass static and known-displacement checks; provide live feedback. |
| `align_session.py` | Merge mapped clocks offline, retain raw provenance, detect gaps and annotate cycles/falls/assistance. No invented poses through occlusion. |
| Existing `analysis.py` + evaluation reports | Extend current within-mode comparisons to cross-method matched trials, fixed-start-frame task metrics and stop metrics. Every attempt and fault remains counted. |
| `replay_motion.py` | Execute submitted targets using the supervisor, start/stop segments and real timing. Existing `replay-plan` only produces a file and always says `hardware_ready: false`. |
| `fit_gait.py` | Fit/validate the compact trajectory and cycle seams; save period, phase, translations/rotations and envelope. Reconstruct held-out targets before optimization. |
| Experimental MJCF + `identify_dynamics.py` | Moving base, actual foot contacts, actuators and frame corrections; fit against physical traces and validate untouched trials. Produce parameter uncertainty and model report. |
| `locomotion_env.py` | Gymnasium tasks, shared observations/actions, rewards, failure reasons, randomization and reproducible reset. Check zero-action stability and invariance to world heading. |
| `optimize_gait.py`, optional `imitate.py`, `train_policy.py` | Baseline search, optional cloning, PPO runs/checkpoints/normalization, evaluation callbacks and resource limits. No real robot endpoint in the simulator training process. |
| `evaluate_policy.py` | Disjoint seeds/conditions, baseline comparisons, exported-policy equivalence, complete trial accounting and physical transfer report. |
| `policy_runtime.py` | Frozen policy inference with identical preprocessing, history and action scaling; deadline monitoring; supervisor controls all targets. |
| `locomotion_service.py` | Finite walk/stop/status operations, unique request IDs, exclusive execution, deadlines, tracking checks and truthful completion/failure results. |
| `speech_tools.py` | Adapter for the recovered speech stack: select Mini, validate intent/arguments, call the skill, acknowledge real outcome and route stop independently of the SLM. |

Project artifacts also include `hardware-profile`, camera calibration, session manifests, measured motion envelope, gait file, fitted model parameters, task/reward config, train/validation/test lists, policy bundle and evaluation report. Unknown motor/tilt/watchdog limits remain unset; the executor must refuse to arm until required limits are measured/configured. Do not fill them from the GIF or simulation defaults.

Tests should target meaningful risks: frame/unit mistakes, kinematic infeasibility, stale inputs, lost ownership, duplicate requests, incorrect distance from rocking, synchronization gaps, missing failures and differing deployment preprocessing. A unit test is not evidence of physical balance.

## 11. Connect your Mac mini/Ubuntu/Temi conversation system

The architectural boundary is:

```text
Your speech → existing speech recognition → existing local SLM / intent router
            → validated “Mini: walk forward” request
            → locomotion service → supervisor → frozen policy → Reachy SDK
                                          ↑
                               joints + IMU + live foot tracker

Stop button / direct stop recognizer → supervisor
Skill result → speech response / conversation with Temi
```

Keep the existing local SLM. Recover its endpoint and the current STT/TTS/robot adapters before replacing any component. There is no need to train that language model on motor trajectories. An exact phrase can also use a simple intent rule while richer conversational tool calling is integrated.

An example **proposed** skill request (not an existing endpoint) is:

```json
{
  "request_id": "unique-per-human-request",
  "robot_id": "reachy_mini",
  "skill": "walk_forward",
  "distance_m": 0.05,
  "max_duration_s": 20
}
```

The service validates fields, current mode, readiness, pose tracking and calibrated workspace. Distance is clamped/rejected against tested capability; the LLM cannot set arbitrary joint targets or enlarge limits. If live distance measurement is unavailable, decline the distance request or offer an explicitly timed burst. Repeated delivery of the same request ID must not start another walk. New movement requests while busy receive a busy result; do not queue surprise movement.

State machine: `IDLE → PREPARING → WALKING → STOPPING → IDLE`, with faults entering `FAULT` and requiring a fresh operator readiness check. A normal stop uses the experimentally verified settling trajectory. A fault cancels new gait targets and invokes the installed, tested hold/stop behavior. Neither instant neutral snapping nor disabling torque is assumed universally stable. Measure what the robot actually does.

Register `walk_forward`, `stop` and `status`. Route a recognized human “stop” directly to the supervisor without waiting for SLM reasoning; retain an independent operator stop and local command-expiry guardian. Measure both recognition delay and physical settling time. On network failure the guardian is the protection, because a disconnected Mac cannot send a stop.

The conversation system must yield head/body control during locomotion. The current official [conversation app](https://github.com/pollen-robotics/reachy_mini_conversation_app) includes motion layers and speech-reactive movement, so adding a walking tool alone does not establish exclusive ownership. Suspend those producers, acknowledge the command, execute, then resume conversational motion only after settling. The official [fully local speech guide](https://huggingface.co/blog/local-reachy-mini-conversation) confirms a local VAD/STT/LLM/TTS backend is possible; it does not identify the version or stack you previously deployed.

Explicitly route the addressed robot. “Temi, move” must not actuate Mini. Initially accept locomotion requests only from the intended human command channel; do not let Mini's own TTS or Temi's conversation trigger a new walk. Use the known audio-source/routing state and push-to-talk if needed rather than assuming acoustic speaker identification works. Robot-to-robot movement delegation can be enabled later as a separate intentional interaction.

Speech integration acceptance tests:

1. In dry-run mode, correct forward intent calls Mini once; ordinary conversation and Temi-addressed commands cause no Mini motion.
2. Duplicate IDs, busy requests, malformed/out-of-range arguments, missing tracker and unavailable robot return explicit results.
3. “Stop” works while the SLM is busy; the operator stop and local expiry still work with disconnected networking.
4. TTS/Temi echo does not initiate motion, and conversational gestures do not write motors during a walk.
5. A physical successful request returns measured travel and completion; an abort returns its actual reason and partial travel. The spoken response does not claim success on a timeout.

## 12. Your next action

For the first session, put **Mini, Quest, the Mac mini and a fixed camera** on the test setup above, with Ubuntu reachable if the old speech stack needs it. Keep the original robot and speech software intact. Have the original Emcee build available if it still runs.

The first session's deliverable is **one instrumented, measurable wobble and a verified way to stop it**. Next comes repeatable replay. Once the model matches those recordings, training has a meaningful target. The final deliverable is a saved policy bundle plus a finite, tested voice skill and physical evaluation results for the stated surface and camera setup.
