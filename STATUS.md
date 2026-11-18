# Reachy Mini — Quest to Locomotion — Status

## November 18, 2026 — repository launch and moving-foot experiment

Current request: inspect the user's Vimeo movement, create a GitHub repository,
and begin learning head-driven penguin-like locomotion with sim-to-real intent.
Latest user correction: **Meta Quest 3S**, not the earlier Quest 3 shorthand.

Repository created: **https://github.com/kadiru/reachy-mini-locomotion** (private).
Local root is this project folder; upstream clones, raw media, real recordings,
calibration and credentials are excluded. Package installation and both CLI
entry points were verified locally. The project virtual environment's packaging
tools were upgraded to support editable `pyproject.toml` installs.

Initial implementation pushed as `c5341b9`. [GitHub CPU CI](https://github.com/kadiru/reachy-mini-locomotion/actions/runs/44390624399)
passed on Linux/Python 3.11, including package installation, a fresh pinned model
checkout/LFS hydration and the test suite. Local Python 3.9 package installation,
both CLI entry points and `pip check` also passed. Local working tree was clean
after the implementation push; this documentation update records the CI receipt.

Reviewed the opening tabletop sequence in `https://vimeo.com/kadiru/demo-reachy-reach`
through the Vimeo player. Head turning/tilting, body rocking and apparent
progress toward the laptop are visible. No metric speed or causal contact-force
claim is supported by that uncalibrated footage. Detailed observations and
coarse timestamps: `docs/VIMEO_OBSERVATIONS_2026-11-18.md`.

New implementation: installable package/CLI; `contact_probe.py`; explicit
uncalibrated contact config; CPU test workflow; four additional tests; learning
roadmap. The derived model preserves the pinned upstream XML, all original
inertias and five closure constraints, adds one free foot joint and a cylinder
contact proxy, and uses configurable unmeasured actuator limits. This supersedes
the historical statement below that no free-foot experiment exists.

Validation: **50 tests passed** locally (all physics tests executed, no skips).
Two-second hold and external-push trials ran at both 2 ms and 1 ms timesteps with
no numerical warnings. Hold drift below 1 micrometre; 2 ms maximum closure error
approximately 1.85 micrometres. The 2 N external push caused about 0.198 mm lateral
travel at either timestep. It is an artificial perturbation, not a gait. Reports:
`experiments/reports/contact-probe-2026-11-18.json`,
`experiments/reports/contact-probe-1ms-2026-11-18.json`, and
`experiments/reports/repository-tests-2026-11-18.txt`.

Commands from this directory:

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion.contact_probe --output experiments/reports/contact-probe-next.json
```

No trained policy, head-actuated simulated gait, new physical trial, robot
actuation or GPU training has occurred. The proxy is not calibrated to the
Wireless underside, battery/COM, surface or motors. Next: measure the underside
and mass; commission the existing recorder; capture synchronized Quest targets,
joints/IMU and external foot pose; implement validated SDK-frame replay in the
free-foot model. `docs/LEARNING_ROADMAP.md` describes the staged experiment.

## Historical status (retained for continuity)

Updated: October 31, 2026 (America/Los_Angeles).
State: First-session software implemented and tested with synthetic camera frames and a mock robot: authenticated Quest bridge, integrated recorder, 3D camera tracking/calibration and robot-local stop guardian. Instrumented physical reproduction and Unity/robot deployment remain pending. No GPU training, real robot actuation, publication or external messaging performed.

## Carrier concept research — Astro versus temi

User requested an investigation into programmable Astro/temi as a carrier and character partner for Mini. Research only; no hardware contacted or modified. Full source ledger and proposed checks: `experiments/reports/astro-temi-interface-research-2026-10-31.json`.

Amazon's [Alexa Web API](https://developer.amazon.com/en-US/docs/alexa/web-api-for-games/alexa-games-about.html) explicitly supports Astro, so saying it has no developer interface would be incorrect. Custom HTML/JavaScript visuals/audio and the [Smart-Motion extension](https://developer.amazon.com/en-US/docs/alexa/web-api-for-games/alexa-games-extensions-smartmotion.html) offer a legitimate limited character-app route. Actual capabilities/choreos must be queried on the device. No public arbitrary driving/velocity or neck-angle SDK, or reproducible Astro rooting procedure, was found. The university SDK announcement does not establish public access. A 2023 Home Assistant owner report describes named-location commands/TTS/routine gestures via unofficial Alexa Media Player. Source inspection of `silas-cole/claude-astro-brain-rider` confirms its control script speaks commands through TTS; its success is not a robot acknowledgment. These paths remain untested on the user's Astro, including custom-face/navigation coexistence.

For direct driving plus expressive UI, recommend temi conditionally: public Android SDK has movement, navigation, screen tilt, speech and callbacks. Current SDK contributor guidance and official REST documentation require PRO entitlement; verify user's installed model/software/subscription first. REST is for higher-level navigation/speech/call sequences. Astro cargo limit is 4.4 lb in its bin; temi's nominal tray limit is 3 kg and assembly-dependent. Mount plus Mini mass/clearance/stability remains unmeasured. No carrier integration was implemented. This exploratory branch does not replace or validate the existing Mini locomotion work.

## Hands-on session — read-only preparation started October 28

User requested the first recording session, beginning with read-only checks and camera calibration. Local checks identify this computer as “Kadir’s Mac mini”; active interface `en0` currently has LAN IPv4 `192.168.68.53` (recheck after a network change). The project environment imports Python 3.9.6, OpenCV 4.12.0 and NumPy 2.0.2 successfully. System-level macOS enumeration finds a USB camera, vendor 1133/product 2081, plus two Continuity Camera entries. The initial sandboxed camera query returned no entries; that was not evidence of an absent camera. No camera frames have been captured.

AVFoundation enumeration reports the USB camera at index 0 and the two Continuity Cameras at indices 1 and 2. Confirm the actual USB view in the calibration preview before saving images. The real Mini address/dashboard URL and camera choice have been requested. No robot endpoint has been contacted, motors commanded, calibration measurements invented, or hardware-verification flags changed. Read-only network/device queries succeeded after sandbox escalation; no approval rejection remains outstanding. Next: obtain the robot address, run `inspect-robot`, identify the USB camera in the preview, capture varied views of the ruler-checked printed checkerboard, then calibrate the fixed world/foot marker geometry. Quest preview and physical control commissioning follow those checks.

## Latest work — first-session software built

User authorized implementation while preparing the hardware. Added `profile.py`, `geometry.py`, `protocol.py`, `robot_io.py`, `guardian.py`, `camera.py`, `session.py`, `first_session.py`, `smoke.py`; direct-LAN Quest sender source `integrations/unity/ReachyRecordingSender.cs`; an explicitly synthetic preview profile; pinned host capture dependencies; printable markers/checkerboard PDFs; and `docs/FIRST_RECORDING_SESSION.md` with exact operator commands.

The new mapping is reconstructed clutched head roll/pitch/yaw plus configurable body yaw, with translation locked and antennas untouched. It is not the original deployed Emcee mapping. UDP packets use HMAC authentication, source sequence/time, tracking validity and receiver challenges. Legacy/invalid/reordered/expired packets cannot actuate. B+trigger is the deadman; grip, release, keyboard, camera loss and timeouts stop and latch. Hardware mode refuses the unverified example profile and synthetic camera calibration/input.

The guardian runs on Mini and writes only to its localhost daemon. It checks installed SDK version, analytical IK, configured joint/pose/rate bounds, tracking error, existing moves and unexpected target changes. A per-trial lease expires without fresh commands; an OS lock prevents a second instance of this guardian for the same local daemon. Other SDK applications must still release the motors. Stop requests submit the measured pose once and revoke the lease; neither torque disabling nor return-to-neutral is automatic. Physical settling/stop latency and daemon-failure behavior are unverified, and analytical IK is not a general mesh collision checker.

Camera calibration uses checkerboard intrinsics and measured marker-to-foot mounting; tracking estimates both world and foot-tag pose in 3D, rejecting small/clipped/ambiguous/missing tags. Original frames are stored before UI annotations with MJPEG compression; saved frame count is verified by decoding. Shared-host monotonic receipt times, source timestamps and robot/Mac clock-offset exchanges are recorded. Derived alignment preserves raw receipt timestamps. Camera exposure and Unity acquisition clocks are NOT hardware-synchronized; no precise exposure-phase claim is made.

**Verification:** 46 unit tests pass (`experiments/reports/first-session-tests.txt`), including prior evidence tests plus authentication/replay protection, release-to-arm, stale data, rate/IK/profile rejection, competing-writer detection, stop holding measured pose, logging failure, calibration and tracking geometry. Full synthetic pipeline passes in `data/synthetic/first-session-smoke-04/report.json`: 40/40 rendered video frames tracked, 33 mock target submissions, robot-local watchdog ACTIVE → STOPPED after input removal. Maximum position error against generated geometry was 0.0044603 m; this is synthetic camera-test error, not physical accuracy or robot travel. The threaded CLI check (`experiments/reports/first-session-cli.json`) confirms 40 saved frames decode, no unarmed commands, camera EOF detection and zero remaining workers at shutdown.

The initial synthetic test exposed ambiguous small-marker poses; the tracker now requires adequate image area and rejects ambiguous solutions, and setup instructions specify a clearer oblique view. Failed developmental reports remain labeled synthetic. OpenCV 4.12.0.88 was installed only in the project `.venv`; NumPy 2.0.2 and Pillow 11.3.0 are pinned in `experiments/requirements-capture.txt`. Unity/C# tooling was unavailable; the sender has not been compiled or deployed to Quest. No camera, Agora channel, robot or Ubuntu job was accessed.

Reproduce from `/Users/kadir/linkedin/projects/reachy-mini-locomotion`:

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion session-smoke --output data/synthetic/new-session-check
PYTHONPATH=src .venv/bin/python experiments/check_session_cli.py
```

First hands-on action: read-only `inspect-robot`, compile/install the new Unity sender in a working copy of Emcee, preview with no actuation, calibrate/verify the fixed camera, and establish the actual hardware profile and stop commissioning procedure. Then run the verified guardian on Mini and an execute session on the Mac. The user is getting hardware ready; no further repeated request for the missing original bridge location is needed.

## Prior planning — complete training and voice runbook

User requested a simple but complete guide to the programs, training tasks and hardware steps needed for “Mini, walk forward,” building on their reported Mac mini/Ubuntu local-SLM conversation system with Temi. `docs/TRAINING_AND_VOICE_RUNBOOK.md` covers hardware preparation; synchronized capture; supervised replay; trajectory fitting; moving-foot/contact/actuator modeling; residual-policy inputs/actions/rewards; proposed staged CPU PPO experiments; held-out evaluation; physical transfer; program responsibilities; and speech skill integration.

At that planning stage, all new trial counts, acceptance targets, residual scales and hyperparameters were explicitly proposals; no new runtime/tracker was yet implemented. The first-session implementation above supersedes the capture-program status. Locomotion environments, trained weights and speech integration remain unimplemented. Initial deployed-policy design depends on a calibrated live external camera; unrestricted walking across rooms/surfaces remains additional work. Speech-triggered replay can be delivered earlier if the physical baseline succeeds, but must remain labeled replay.

The prior speech stack has not been recovered. Accessible recent/archived task listings did not identify it. Read-only inspection of the local `woz-demo-stack-main` README and relevant source found a GO2/SO-101 demo, not evidence of the reported Temi/Reachy SLM system. No external project files or running services were changed. Recover deployed service/source/version information on the actual Mac mini/Ubuntu when available; do not assume a new official conversation-app version matches the old installation.

Integration requirements: exclusive head/body motor ownership; suspend conversation gestures during walking; a robot-local expiry/guardian for network loss; deterministic finite requests with deduplication; direct human-stop handling outside SLM reasoning; explicit Mini/Temi routing and rejection of self/TTS-triggered motion. Motor/tilt/timeout limits must be established from the installed hardware and measurements, not copied from GIF/model defaults. The stock model's fixed foot remains a blocker for credible locomotion RL.

## Latest evidence — user GIF

User supplied `/Users/kadir/Downloads/1777717739273.gif`, describing Quest 3 controller motion that made Reachy inch forward like a penguin. Preserved an unmodified copy under `media/user-evidence/2026-10-28-quest-wobble/original.gif`; SHA256 `188785a740e8613d9d3829ca45b2702cfc95783f56e609038703073ffbe51b14` matches the source. 800×800, 63 frames, 80 ms/frame, 5.04 s GIF loop. Playback timing is not verified source-camera timing.

All frames were visually inspected. Head turning/tilting accompanies controller orientation changes; the whole body rocks. The base appears to advance toward the laptop in the later segment. One visible topple develops at frames 18–22 (GIF 1.44–1.76 s), the operator reaches for the fallen robot at 28–29, and the transition 29→30 is a reset/discontinuity. Do not claim autonomous recovery, infer a fall rate, or measure displacement across that boundary or the GIF loop. A cable is visible; function and tension remain unknown. Camera movement, no spatial calibration and missing commands/states preclude a reliable distance/speed claim.

Evidence status is now **user-supplied physical video, visually reviewed**, beyond the prior verbal report; it is not an independently reproduced or instrumented experiment. Notes: `docs/GIF_OBSERVATIONS_2026-10-28.md`; machine-readable annotations/metadata and full-frame contact sheets are beside the preserved original. Existing synthetic fixture results remain synthetic and separate.

New reproducible command:

```sh
.venv/bin/python experiments/inspect_user_gif.py /Users/kadir/Downloads/1777717739273.gif media/user-evidence/2026-10-28-quest-wobble
```

Pillow 11.3.0 was installed only in project `.venv`; `experiments/requirements-media.txt` records it. The extractor was run successfully and checked the original's hash. No learned tracking or fabricated base-pose measurements were added. Next capture should preserve coupled turning/tilting and document the transition into tipping, with external foot tracking and the original cable condition recorded.

## User clarifications preserved

- Robot is **Reachy Mini Wireless**, used on a **flat surface**.
- User made it travel by fast head swinging, wobbling left/right. This is an observed behavior reported by the user; it is not dismissed because the stock product is described as stationary.
- User supplied **https://github.com/kadiru/Emcee** as the likely Quest teleop source.
- User does not remember where the robot-facing bridge ran; it may have been custom code or existing Reachy software. Do not repeatedly ask the same location question. Work with the original machine/robot when available.
- A tabletop and cable are visible in the supplied GIF. Exact surface material, attachments, cable function/tension, installed SDK version and quantitative motion results are still unknown. “Can walk anywhere” has not yet been evaluated as directional control across surfaces.

## Concrete evidence and outputs

**Implemented locally:** JSONL recording contract/validator; Emcee input parser and callback capture hook; read-only HTTP state logger; per-cycle external base displacement, forward/lateral motion, yaw and repeatability analysis; gap rejection; falls/abort/assistance accounting; matched-condition independent-trial comparison; offline target replay schedule generation. Code: `src/reachy_locomotion/`. Recorder/analyzer uses standard-library Python; optional MuJoCo dependencies are isolated in `.venv`.

**Recovered from Emcee source:** pinned commit `76ab2fdef8e872d36c0bfd0c866570e512a2aaad`. `unityWin/EmceeXR/Assets/AgoraTemiViewer.cs` sends right-controller orientation, trigger/A/B/grip and thumbsticks via Agora, default 50 Hz. `pythonReachy/reachy_teleop_rtc.py` only decodes/prints. No actual SDK target mapping was found in the current snapshot. Remote branch check found only main; older history and deployed/uncommitted files were not searched. No Agora connection was made. The sender keeps button states on tracking loss; recovering a receiver must include an independent stale/tracking-invalid stop condition.

**Official model inspected, not a locomotion simulation result:** SDK commit `a9a2a42adcc1abc7d9c90d390889bbebf930c96c`; MuJoCo 3.3.7 successfully compiled the unchanged empty scene. `nq=37`, `nv=30`, `nu=9`, no free joints, zero root-foot collision geoms, root subtree mass 1.17652824 kg, default floor friction `[1,0.005,0.0001]`, all nine selected position actuators force-limited to `[-20,20]` via `perfect_actuator`. The stock model cannot reproduce free-foot travel; base contact, realistic actuators and frame transforms need work before RL. Result: `experiments/reports/stock-mujoco-audit.json`. No rollout/training was run.

**Motor/balance signals researched:** inspected standard state routes expose measured and target joints/poses; Wireless IMU provides acceleration, angular rate, quaternion and IMU temperature. Motor current/load/temperature and contact forces are not in the inspected full-state API. Actual installed-daemon availability and IMU-to-foot calibration remain unverified. Head FK and body-yaw angle are not base displacement.

**Synthetic/mocked only:** `data/synthetic/known-answer.jsonl` has 131 records and three deliberately invented 10 mm cycles. It tests math and formatting, not robot performance. HTTP capture tests use mocked responses. `experiments/reports/synthetic-replay-plan.json` is a schedule, not executed replay, and has `hardware_ready: false`. No external video/tag tracker, synchronized capture integration, real daemon session or hardware replay has been validated.

**Original scaffold tests:** 14 unit tests passed; evidence in `experiments/reports/tests.txt`. They are included in the current 46-test suite above. Includes zero-net wobble, direction-frame transform, tracking gaps, no extrapolation, head/base separation, falls, incomplete cycles, timestamp/nonfinite rejection, no overwritten recordings, read failures and replay provenance. Fixture, source-ledger and model-report consistency checks passed.

**Public sources:** `media/source-ledger.json` has 12 entries: three verified written cannot/can't-walk statements, a qualified HN comparison, two original LinkedIn comments supporting prior motion attempts, four YouTube leads, one unverified X context link and one stale search-snippet lead. **Zero verified spoken clips or video timestamps.** Videos could not be fetched by the web tool; no videos were downloaded/reposted and no creators were contacted. `media/SHOT_LIST.md` has the reveal sequence, with explicit teleoperation/replay/learned labels.

## Runnable commands

Working directory: `/Users/kadir/linkedin/projects/reachy-mini-locomotion`.

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion validate data/synthetic/known-answer.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion analyze data/synthetic/known-answer.jsonl --output experiments/reports/synthetic-analysis.json
PYTHONPATH=src .venv/bin/python -m reachy_locomotion replay-plan data/synthetic/known-answer.jsonl --output experiments/reports/synthetic-replay-plan.json
PYTHONPATH=src .venv/bin/python -m reachy_locomotion audit-sim --output experiments/reports/stock-mujoco-audit.json
```

After filling out `data/session-template.json` and identifying the actual daemon address (not run on hardware here):

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion capture-state --url http://REPLACE_WITH_ROBOT_IP:8000 --metadata data/session-template.json --output data/physical/first-state-check.jsonl --seconds 10 --hz 20
```

This is read-only; it does not wake the robot or enable motors. State polling alone neither captures every submitted command nor measures world displacement. Coordinate command/state/video capture through one Recorder and measured clock mappings before claiming synchronization.

## Immediate next action

Recover the robot-facing bridge from the original Wireless robot or demo computer; preserve its code and installed versions. If unavailable, label a reconstructed mapping explicitly and verify axes/neutral/limits at low amplitude. Then record one short rest → manual wobble → rest trial with simultaneous submitted commands, measured joints/IMU, fixed external foot tracking and side-view video. Quantify net base travel against static drift/uncertainty before proceeding to supervised replay.

The reconstructed first-session implementation is now ready for that commissioning workflow; use `docs/FIRST_RECORDING_SESSION.md`. Hardware replay and trained policies are still later milestones.

No permission or auto-review rejection remains outstanding. Missing hardware access and the deployed mapping limit the physical next step; the local tools are ready for instrumentation work.

## Next milestones and references

1. One measured teleop reproduction; identify rocking vs sliding and any cable/surface contribution.
2. Supervised replay; proposed 5 independent trials × 10 cycles, include all failures and compare matched conditions.
3. Compact imitation/parameter optimization after calibrated simulator replay.
4. RL only after free-base/contact/actuator model validation; no shared-GPU work until coordinated with the existing humanoid job owner.

`README.md` has usage. `docs/RECOVERY.md`, `docs/RECORDING_SCHEMA.md`, `docs/EXPERIMENT_PLAN.md` and `docs/SIM_AUDIT.md` contain sources, capture design, reward/evaluation definitions and limits.

Reference downloads are project-local: Emcee (~1.2 GB, vendor assets included) and Reachy SDK (~40 MB after 65 official LFS assets were hash/size verified). Initial MuJoCo compilation failed on unhydrated LFS meshes; helper `experiments/fetch_mujoco_assets.py` resolved this. `experiments/requirements-audit.lock` records the isolated environment. Sibling projects and existing humanoid training were untouched.
