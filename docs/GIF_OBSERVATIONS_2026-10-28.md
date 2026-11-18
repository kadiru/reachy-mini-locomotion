# User GIF: controller-driven penguin-like motion

Reviewed October 28, 2026 (America/Los_Angeles). This is **user-supplied physical-video evidence of teleoperation**, now inspected across all 63 frames. It is not synthetic and not a new hardware experiment conducted by this project.

The clip makes the reported mechanism substantially clearer: changing controller orientation accompanies rapid head turning/tilting, the entire body leans and rocks, and the base appears to progress across the tabletop, particularly in the later sequence toward the laptop. The user's description of inching forward like a penguin fits the visible motion. No distance, speed or clean-trial success rate is estimated from this uncalibrated clip.

## Provenance

- User source: `/Users/kadir/Downloads/1777717739273.gif`.
- Unmodified project copy: `media/user-evidence/2026-10-28-quest-wobble/original.gif`.
- SHA256: `188785a740e8613d9d3829ca45b2702cfc95783f56e609038703073ffbe51b14`.
- 800 × 800 pixels; 63 GIF frames, each held 80 ms; 5.04 seconds per GIF loop. These describe GIF playback, not verified original camera acquisition or real-time execution speed.
- User identifies the hardware as Reachy Mini Wireless and Meta Quest 3. A cable is visible; that does not change the reported hardware version. Its function, tension and effect on motion are unknown.
- Controller commands, measured motor states, IMU logs, spatial calibration and source-camera timestamps are absent.

## Frame observations

Frame indices are zero-based. Times below are **GIF-relative playback times**, not synchronized robot timestamps.

| Frames | GIF time | Visible evidence | Interpretation limit |
|---|---|---|---|
| 0–17 | 0.00–1.36 s | Operator changes controller orientation repeatedly; head alternates turned/tilted poses; body rocks sideways. | Exact controller axes, SDK mapping, command amplitude and phase delay cannot be recovered from these images alone. |
| 18–21 | 1.44–1.68 s | Body inclination increases markedly toward image left; foot support is visibly changing. | Consistent with tipping about the base edge, but exact contact forces/support patches are hidden. |
| 22–27 | 1.76–2.16 s | Robot lies on its side; one visible topple has occurred. | Count this as one observed fall, not evidence for a statistical fall rate. |
| 28–29 | 2.24–2.32 s | Operator reaches toward and appears to contact the fallen robot. | Human intervention is present; the next upright pose must not be called autonomous recovery. |
| 29→30 | 2.32→2.40 s | Abrupt transition from fallen/intervention scene to upright robot and changed framing. | Treat as a reset/discontinuity. Do not fit displacement, cycle phase or recovery time across it. |
| 30–62 | 2.40–4.96 s | Repeated controller/head motion and whole-body rocking continue; base appears to approach the laptop. No additional fall is visible before the clip ends. | Camera moves; later proximity to the laptop may change contact conditions. No complete standardized trial or metric displacement can be established. |

The last frame is held through 5.04 s, then the GIF loops to frame 0. That loop boundary is not a real reset or reverse movement. There is no basis for computing “one failure out of two trials”: both visible segments may be excerpts.

## What this changes for the build

1. Preserve this as a positive demonstration of the user's **teleoperated wobble locomotion attempt**, with a visible failure example. The prior evidence state was user recollection; it now includes inspectable physical footage.
2. Investigate coupled head **turning and tilting**, rather than reducing the motion to one roll sine wave without recovering the actual controller mapping. Emcee's degrees and calibration remain useful, but images do not provide robot targets.
3. The working mechanism is head-driven rocking with changing support and possible slip. Alternating unloading, friction and timing could turn the wobble into net progress; exact causal contributions remain to be tested. The cable's contribution is unknown, not established.
4. The stock simulator's fixed foot and absent foot collision geometry directly exclude the large body rocking seen here. A suitable model must support free-base tilt, rim/pad contact, realistic head inertia and a fall like frames 18–22, as well as successful progress.
5. Begin the next capture with a fixed external camera and foot marker, a clear travel area, recorded cabling condition and synchronized commands/joints/IMU. Record the successful wobble and the approach to excessive tilt so a replay can retain travel while avoiding the observed tipping mode.

No guessed frame positions have been inserted into the physical JSONL dataset. All quantitative displacement analysis still requires a calibrated measurement. No RL training or robot commands were started from this clip.

## Reproduce frame extraction

From `/Users/kadir/linkedin/projects/reachy-mini-locomotion`:

```sh
.venv/bin/python -m pip install -r experiments/requirements-media.txt
.venv/bin/python experiments/inspect_user_gif.py /Users/kadir/Downloads/1777717739273.gif media/user-evidence/2026-10-28-quest-wobble
```

The extractor checks an existing original's hash and refuses to replace it with a different input. It saves source metadata, selected full-resolution frames, a 24-frame overview and three contact sheets covering all frames. It does not calculate pose or infer physics.
