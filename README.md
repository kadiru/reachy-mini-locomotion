# Reachy Mini — Quest to Locomotion

Learn head-driven, penguin-like locomotion from **Meta Quest 3S** demonstrations,
then evaluate sim-to-real transfer on Reachy Mini Wireless.

The intended gait library has [two styles](docs/GAIT_STYLES.md): rapid small
steps and larger alternating-edge steps driven by sway, head turning and braking.
Both have simulation candidates. The latest balance-first controller adds
bilateral recovery and slower, larger step pairs; the earlier fast modes remain
available. Generalization beyond the tested conditions remains incomplete.

Reference: Kadir's [reachy reach video](https://vimeo.com/kadiru/demo-reachy-reach).
The opening sequence shows controller-driven head motion, whole-body rocking
and apparent base progress. [Video observations](docs/VIMEO_OBSERVATIONS_2026-11-18.md)
separate what is visible from what still needs measurement.

The newest simulations cover a walk at one directional swing per second and
balance under direct side hits and overhead drops. The drop film follows all
78 episodes through recovery, a flat landing, or a topple. The Google Slides
presentation includes the training methods, sensor inputs, walking and balance clips.
[Walking at operator cadence](docs/TELEOP_CADENCE.md) ·
[Direct side hits](docs/SIDE_BALL_IMPACTS.md) ·
[One through ten overhead balls](docs/OVERHEAD_BALL_DROPS.md) ·
[Google Slides presentation](https://docs.google.com/presentation/d/1PVjdl7sI0adp9ynv8d7YQ7XYvui_ADdw98XGs95qBeU/edit).

The latest slow gait covers 25.65 mm in 24 head swings and brakes on alternate
loaded edges in 21 of them. A faster variant covers 37.61 mm in the same time.
The comparison includes every matched trial, including the 27.19° heading miss.
[Watch the alternating steps](media/simulation/alternating-edge-brakes.mp4) ·
[Walking and balance follow-up](docs/EDGE_WALK_AND_BALANCE_FOLLOWUP.md).

After retraining on that heading miss, Reachy passed all ten retests, ten
new floor/motor trials and two 40-second walks. Its fastest new trial covered
39.60 mm in 16 swings, with one rim lifting 29.33 mm.
[Watch the fastest fresh edge walk](media/simulation/edge-walk-fastest-fresh.mp4).

**Faster ball impacts and 40° catches (November 20).** A roll-rate fallback
kept Reachy upright after a **9.248 m/s** bounce by giving up the edge and
settling flat in **0.72 s**. A **9.373 m/s** bounce toppled the same simulated
robot. The fastest hit followed by sustained edge balance was **1.200 m/s**.
A separate head maneuver, triggered after measuring 40°, returned prepared
outward-moving starts to flat from **40.45° left and 40.36° right**.
[Watch the fastest bounce and topple in slow motion](media/simulation/fast-ball-catches.mp4) ·
[Watch the measured 40° catches](media/simulation/measured-40-catches.mp4) ·
[Results, every trial on video and replay instructions](docs/HIT_SPEED_AND_40_DEGREES.md).

**Ball-impact recovery (November 20):** two complete simulations now start flat,
lift onto the left/right edge, withstand a rolling ball from each side, and
return flat without falling. All **four physical bounces** regain qualified
balance within **0.32–1.34 seconds**. The balls are 8 cm diameter, 10 g and
released at 0.3 m/s. The waist target and motor limits are unchanged.
The frozen impact sensitivity suite passes **7/28**; stronger hits, timestep
changes and added delay still fail. The new gyro processing is an uncalibrated
simulation assumption, not the stock robot SDK. **140 tests pass.**
[Watch both complete sequences at real speed](media/simulation/ball-balance-recovery-2026-11-20.mp4).
[Method, qualification failures and replay](docs/BALL_BALANCE_2026-11-20.md).

**Front/back balance (November 20):** from an explicitly raised posture, the
new controllers hold the front/back edge for **19.76/19.54 seconds** in
20-second simulations near ±32° pitch. Natural lift-and-catch from flat is
still unresolved. The frozen hold policies pass **9/18** sensitivity checks;
noise and mass variation remain failures. Waist target and motor limits are
unchanged. **131 tests pass.**
[Watch the raised-start holds](media/simulation/foreaft-balance-2026-11-20.mp4).
[Method, failures and replay](docs/FOREAFT_BALANCE_2026-11-20.md).

**Sustained edge balance (November 20):** both sides lift from flat, arrest the
rock and balance at about **31–32°**, with one rim about **58–60 mm** above the
other. The nominal 60-second trials contain **57.36/56.74 seconds of continuous
edge hold**. Waist target and motor limits are unchanged. This is an
experimental learned feedback skill: sensor-noise/delay checks and several
timestep checks still fail. It is not ready for a physical robot. **123 tests pass.**
[Watch both sides at real speed](media/simulation/edge-balance-bilateral-2026-11-20.mp4).
[Training, successful trials and failed checks](docs/EDGE_BALANCE_2026-11-20.md).

**Sustained penguin gait (November 20):** **633.4 mm in 163.26 s of stepping**,
**3.88 mm/s**, over **20 complete pairs** with no falls or guard stops and
8.29° maximum cycle-end heading. This is **5.14% faster** than the previous
policy over the same 20-pair check. Both starting sides pass at 0.25 and 0.125 ms.
All **42 known single-pair cases** and **six difficult four-pair cases** pass;
the new frozen four-pair split passes **11/16** (two tilt guards and three
turn/progress failures). Robustness across new conditions is still incomplete.
Waist target and motor limits are unchanged; **110 tests pass**.
[Watch the best real-time video](media/simulation/sustained-penguin-best-2026-11-20.mp4).
[Training, qualification methods and limitations](docs/SUSTAINED_PENGUIN_2026-11-20.md).

**Faster feedback gait (November 19):** retraining the failures and shortening
phase pauses gives **112.1 mm in 30.44 s of stepping**, **3.68 mm/s**—**25.8% faster**
than the preceding balance gait—with 15.3° body roll and 7.52° maximum pair-end
heading. All **26 previously seen cases** now pass; the four formerly failing
cases also pass at 0.125 ms. New frozen qualification passes **14/16**: one tilt
guard and one heading/progress failure remain. Waist target and physical motor
limits are unchanged. **105 tests pass**; no hardware transfer is claimed.
[Watch the best real-time video](media/simulation/faster-penguin-best-2026-11-19.mp4).
[Training, limits and complete results](docs/FASTER_PENGUIN_2026-11-19.md).

**Bilateral feedback follow-up (November 19):** delayed roll/rate feedback and
progress-based primitive selection produce **88.6 mm forward in two pairs**
(eight rock/pivot bursts), **15.3° bilateral roll**, and **0.60° maximum pair-end
heading error**, with a fixed zero waist target and unchanged motor limits.
The recording lasts 32 s; progress speed is 2.93 mm/s through the last step.
Recovery passes **26/26** checks. Isolated pivots and composed pairs pass **20/20
original development checks** at both 0.5 and 0.25 ms. Broader frozen qualification
passes **13/16 pivots and 13/16 walking trials**: two new conditions hit the tilt
guard, and one pivot turns too little on its edge. All four nominal 0.125 ms
checks pass; the two-pair replay at that timestep advances 89.2 mm.
**Robustness across all stress conditions is not yet achieved.**
[Watch bilateral recovery → penguin steps](media/simulation/bilateral-penguin-2026-11-19.mp4).
[Method, complete outcomes and reproduction](docs/BILATERAL_BALANCE_2026-11-19.md).

**Earlier primitive curriculum (November 19):** before further walking optimization,
learn sideways lift, recovery and edge pivots. With the waist holding a constant
zero target, the selected simulation recovers from **30.03° roll / 55.59 mm
opposite-rim height difference**, settling with **2.82 mm center drift**. A
compound head turn/stop produces **17.78° final base rotation**, including
**14.81° during edge support**, then settles upright. The same launch without
the recovery response eventually falls. These nominal outcomes persist at
0.5, 0.25 and 0.125 ms physics timesteps. Fresh model variations still cause
falls and weak pivots; this is not robust walking or a hardware-ready policy.
[Watch fall → recovery → edge pivot](media/simulation/primitive-curriculum-2026-11-19.mp4).
[Measurements, comparisons and failure map](docs/PRIMITIVE_CURRICULUM_2026-11-19.md).

**Large-rocking experiment (November 19, 2026):** **1.004 m in 20.16 s**,
**49.81 mm/s**, **15.94 mm per full left/right cycle**, and **8.56° peak body tilt**.
Every completed cycle stays within **11.58°** of startup heading in the saved
trial, with no fall. At 0.5 ms the stride is 14.65 mm and cycle-end heading stays
within 15.18°. This misses the original 15 mm target at that finer timestep.
The selected controller stops on an unreachable head target after 38.04 measured
seconds in the longer test; it is **not qualified for continuous walking**.
Fresh model variations give 8/16 completed runs and 0/16 heading passes; the
other eight stop at the joint-limit guard. Hardware transfer remains unverified.
[Watch the real-time demonstration](media/simulation/rocking-steps-2026-11-19.mp4).
[Results, failed checks and reproduction](docs/LARGE_ROCKING_2026-11-19.md).

**Preserved rapid mode:** the larger-yaw headshake moves
**114.9 cm in 20.02 simulated seconds**, **19.1% faster** than the preceding
48.19 mm/s gait, with **14.4°** maximum heading change. It remains upright for
120 seconds, moving 6.75 m while staying within 15.2°. Reducing the physics
timestep from 1 to 0.25 ms changes speed by 0.6%. Both physical motor limits
and software command limits are unchanged; swing amplitude and timing enable
the gain. [Watch the real-time simulation](media/simulation/faster-headshake-2026-11-19.mp4).
[Full results and reproduction](docs/FASTER_HEADSHAKE_2026-11-19.md).

**Fast rocking steps, with no airborne flight observed.** All 16 fresh stress
runs stay upright, but only 6/16 remain within 20°; the worst turns 114°.
No physical transfer has been verified. Motor/contact response, software command
profiles and the assumed heading estimator remain uncalibrated.
[Motor sources and assumptions](docs/MOTOR_LIMITS.md).

The preceding [48.19 mm/s speed-first result](docs/SPEED_FIRST_2026-11-19.md)
and [video](media/simulation/speed-first-2026-11-19.mp4) remain available.
The earlier stricter-heading objective and its unresolved stress results are
preserved in the [heading-feedback report](docs/HEADING_FEEDBACK_2026-11-19.md).
The earlier open-loop result was 36.4 cm in 20.52 s; its
[results](docs/PROGRESS_TRAINING_2026-11-19.md) and
[video](media/simulation/progress-fast-2026-11-19.mp4) remain available. The
[original 1.2 Nm model comparison](docs/FORWARD_PROGRESS_COMPARISON_2026-11-19.md)
explains the first, overly optimistic actuator assumption.

## Start here

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[capture,sim]'
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m reachy_locomotion validate data/synthetic/known-answer.jsonl
```

Python 3.9–3.12; use the project-local environment. The contact physics tests
also need the pinned upstream model: follow [CONTACT_PROBE.md](docs/CONTACT_PROBE.md)
to fetch it and run a CPU-only experiment. No robot or GPU is needed.

- [Learning roadmap](docs/LEARNING_ROADMAP.md): demonstrations → replay → model fitting → compact gait learning → residual RL → supervised transfer.
- [First recording session](docs/FIRST_RECORDING_SESSION.md): existing Quest bridge, calibration and robot-local guardian.
- [Moving-foot experiment](docs/CONTACT_PROBE.md): derived official model, explicit assumptions and reproducible results.
- [Head-sway experiment](docs/SWAY_EXPERIMENT_2026-11-18.md): official IK, roll/yaw controller, search results and rendered motion.

## Original first milestone

First milestone: recover the user's observed Wireless Reachy Mini locomotion on a flat surface, caused by rapid head swinging and left/right wobbling; record it, measure base displacement, then reproduce it by replay.

The user's observation is retained and now supported by a supplied GIF showing controller-driven head motion and whole-body rocking, with apparent base progress. One topple and a reset/discontinuity are visible. Directional control, repeatability, metric speed, fall rate and autonomous execution remain unmeasured. There are no instrumented physical command/state recordings yet. See `docs/GIF_OBSERVATIONS_2026-10-28.md` for frame-by-frame notes and provenance.

## What is implemented

- First-session Quest-to-Reachy bridge: authenticated direct-LAN Unity sender source, clutched task-space mapping, explicit preview/record-only/execute modes, and a robot-local expiring guardian with IK/rate/ownership checks and latched stops. Hardware and Unity compilation remain unverified.
- Integrated camera/Quest/command/state recorder; 3D world/foot marker tracking; camera/mount calibration; printable PDFs; guardian clock-offset estimates and receipt-time alignment. Camera exposure and Quest acquisition clocks are not hardware-synchronized.
- JSONL recorder with a shared monotonic clock; separate Quest input, submitted robot target, measured state, external base pose, video index and events.
- Parser and callback hook for the actual Emcee Quest packets.
- Read-only HTTP state capture; no motor enable, waking or command writes.
- Cycle analysis, conservative tracking-gap rejection, forward/lateral displacement, repeatability, fall accounting and matched-condition trial comparison.
- Offline replay schedule generation from submitted SDK targets. It cannot actuate hardware.
- CPU-only model audit, moving-foot contact probe and head-actuated sway controller. Scripted exploration, CEM gait-parameter training and bounded heading feedback are implemented; RL training is not.
- Separate simulation curriculum for intentional sideways falls, delayed-roll recovery and head-driven edge pivots, with bounded fixed-target or unpowered waist modes and material-point slip diagnostics.
- Original-source ledger and reveal shot list. No verified spoken-video timestamps yet.

**Start with `docs/FIRST_RECORDING_SESSION.md`.** It has the exact commands, Unity setup, camera procedure and stop keys. The synthetic end-to-end test passes with a mock robot; no physical actuation has been attempted. The supplied profile is explicitly a preview example and cannot arm hardware.

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion session-smoke --output data/synthetic/my-session-check
```

Use a new output directory. Current validation: **89 local tests pass**; the [CPU workflow](https://github.com/kadiru/reachy-mini-locomotion/actions/workflows/tests.yml) runs the suite on GitHub. The latest local log is `experiments/reports/repository-tests-primitives-2026-11-19.txt`. The earlier full synthetic UDP/HTTP/video pipeline and threaded CLI loss/shutdown check also passed; their generated captures remain local-only.

## Run locally

Run commands from the repository root (this workspace: `/Users/kadir/linkedin/projects/reachy-mini-locomotion`). The original recorder/analyzer needs only the Python standard library. The first-session tools additionally use NumPy/OpenCV/Pillow, pinned in `experiments/requirements-capture.txt`. `.venv` is project-local; optional MuJoCo supports `audit-sim` and the contact probe. Physical guardian execution uses the robot's installed, version-matched Reachy SDK; do not upgrade that installation from the host requirements file.

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion validate data/synthetic/known-answer.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion analyze data/synthetic/known-answer.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion replay-plan data/synthetic/known-answer.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion audit-sim --output experiments/reports/stock-mujoco-audit.json
```

To generate a new fixture, use an unused filename; recordings never overwrite existing files:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion synthetic data/synthetic/another-fixture.jsonl
```

The fixture deliberately invents 10 mm forward travel per cycle. It is a numerical test, not a prediction or robot result. A mock HTTP response tests capture; it is not a daemon integration test.

After filling in `data/session-template.json` with a unique ID, installed SDK version, known setup, and actual daemon address, read-only capture is available:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion capture-state --url http://REPLACE_WITH_ROBOT_IP:8000 --metadata data/session-template.json --output data/physical/first-state-check.jsonl --seconds 10 --hz 20
```

This command has not been run against hardware. A state-only recording cannot measure displacement or reconstruct every command. An older installed daemon may lack these fields/routes. Start by identifying its version; preserve the existing working setup instead of updating it mid-recovery.

For recorded, independent trials with identical setup metadata:

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion compare data/physical/trial-01.jsonl data/physical/trial-02.jsonl --output experiments/reports/replay-comparison.json
```

Different conditions or duplicate session IDs are rejected. `sample_sd_forward_m` is within-session cycle variability; `between_trial_sd_m_per_cycle` is variability of independent trial means.

## Evidence and next work

- `docs/FIRST_RECORDING_SESSION.md`: runnable software, setup, calibration, stop controls, test evidence and hardware commissioning boundaries.
- `docs/TRAINING_AND_VOICE_RUNBOOK.md`: complete proposed hardware workflow, software work packages, simulation/training tasks, transfer criteria and integration with the user's existing Mac mini/Ubuntu speech system. Clearly separates existing tools from programs still to implement.
- `docs/RECOVERY.md`: Emcee findings and accessible motor/balance signals.
- `docs/GIF_OBSERVATIONS_2026-10-28.md`: supplied physical-video evidence, fall/reset annotations and implications for the mechanism.
- `docs/RECORDING_SCHEMA.md`: recording contract, integration hook and synchronization.
- `docs/EXPERIMENT_PLAN.md`: first capture, replay criteria, imitation/optimization/RL progression.
- `docs/SIM_AUDIT.md`: pinned source audit, measured model parameters and changes required before RL.
- `media/source-ledger.json`: verified text versus unverified clip leads.
- `media/SHOT_LIST.md`: reveal with truthful control-mode labels.
- `STATUS.md`: handoff, checks and immediate next action.

Reference snapshots are local-only under `experiments/upstream-emcee` and `experiments/upstream-reachy-mini`; they are excluded from Git along with raw video, physical recordings, calibration, credentials and generated captures. Emcee includes large vendor assets (~1.2 GB). Fetch the pinned model using the contact-probe guide on a fresh clone. Nothing has been sent to Agora, a robot, or a creator.
