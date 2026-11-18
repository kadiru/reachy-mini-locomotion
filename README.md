# Reachy Mini — Quest to Locomotion

Learn head-driven, penguin-like locomotion from **Meta Quest 3S** demonstrations,
then evaluate sim-to-real transfer on Reachy Mini Wireless.

Reference: Kadir's [reachy reach video](https://vimeo.com/kadiru/demo-reachy-reach).
The opening sequence shows controller-driven head motion, whole-body rocking
and apparent base progress. [Video observations](docs/VIMEO_OBSERVATIONS_2026-11-18.md)
separate what is visible from what still needs measurement.

**Current state (November 18, 2026):** CPU gait training now uses published
Dynamixel torque/speed data, motor-specific limits, command smoothing/latency,
and corrected total mass. [Motor sources and assumptions](docs/MOTOR_LIMITS.md)
and [training protocol/results](docs/MOTOR_TRAINING_2026-11-18.md) distinguish
measured manufacturer specifications from uncalibrated model parameters.
The refined candidate advances **3.2 cm in 13 simulated seconds**, with about
64% lower sampled body acceleration than the first trained candidate. It passes
our walking check in **8/12 fresh held-out cases** (up from 2/12); all 12 move
forward. [Watch the real-time simulation](media/simulation/motors-trained-2026-11-18.mp4).
Sim-to-real transfer remains unverified. The historical 15.8 cm demo used an
unrealistic 1.2 Nm head-motor cap and is retained only as an exploratory result.

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
- CPU-only model audit, moving-foot contact probe and head-actuated sway controller. Scripted exploration and CEM gait-parameter training are implemented; RL training is not.
- Original-source ledger and reveal shot list. No verified spoken-video timestamps yet.

**Start with `docs/FIRST_RECORDING_SESSION.md`.** It has the exact commands, Unity setup, camera procedure and stop keys. The synthetic end-to-end test passes with a mock robot; no physical actuation has been attempted. The supplied profile is explicitly a preview example and cannot arm hardware.

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion session-smoke --output data/synthetic/my-session-check
```

Use a new output directory. Current validation: **63 local tests pass**; the [CPU workflow](https://github.com/kadiru/reachy-mini-locomotion/actions/workflows/tests.yml) runs the suite on GitHub. The latest local log is `experiments/reports/repository-tests-motors-final-2026-11-18.txt`. The earlier full synthetic UDP/HTTP/video pipeline and threaded CLI loss/shutdown check also passed; their generated captures remain local-only.

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
