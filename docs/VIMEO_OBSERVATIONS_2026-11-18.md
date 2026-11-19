# Reachy reach — video review

Reviewed November 18, 2026 (America/Los_Angeles).

Source: Kadir Uyanik, [reachy reach](https://vimeo.com/kadiru/demo-reachy-reach).
The page resolves to [video 1243537656](https://vimeo.com/1243537656/5c824721ae).
The player displays approximately 1:25 total duration. The user identifies the
controller/headset setup as **Meta Quest 3S**; this corrects the earlier project
notes that called it Quest 3. Hardware model is user-provided information, not
an identification inferred from the headset's appearance.

## Direct observations

The opening tabletop sequence was played and sampled through the Vimeo player
around displayed times 00:00, 00:03, 00:04 and 00:05. These are coarse player
timestamps, not synchronized acquisition times or exact event boundaries.

- Handheld controller orientation changes accompany head turning and tilting.
- The torso/foot assembly rocks as well; this is not only a stationary torso
  with an expressive head.
- By approximately 00:05 the base appears closer to the laptop than at the
  opening. This is consistent with the user's reported penguin-like progress.
- A cable and nearby laptop are visible. The camera/framing moves, the underside
  is partly occluded, and no scale or motor telemetry is available.
- By approximately 00:07 the edit has cut to other footage. Around 00:14–00:18,
  the operator is visible wearing a headset and holding a controller.

The review supports **teleoperated rocking and apparent travel**, not a numerical
speed, autonomous gait, identified contact sequence, or proven force mechanism.
The earlier GIF's fall/reset annotations remain specific to that GIF. They are
not assigned to timestamps in this edited Vimeo video.

## Hypothesis to test

Head motion moves the center of mass and generates reaction forces/torques;
alternating support on the base rim, with timed yaw/pitch/roll and slip, may
produce net translation. This is a working hypothesis, not a causal conclusion
from these images. A perfectly symmetric sway need not advance, so recover the
coupled motion and its timing rather than assuming a roll-only sine wave.

Record foot pose with a fixed external camera, joint targets and measured
joints/IMU together. Measure the actual contact footprint and document cable
slack, surface and robot mass. Preserve successes and failures. The video is a
qualitative reference; it is not itself a command-labeled imitation dataset.

The original video and the earlier local GIF are not redistributed in this
repository. No inferred metric poses were added to physical recordings.

## November 19 follow-up

The user clarified two desired styles: rapid small steps and larger deliberate
edge-supported steps with pronounced torso tilt and left/right rotation.
See [GAIT_STYLES.md](GAIT_STYLES.md). This is user-provided intent and mechanism
description, not a newly measured physical result.

A fresh playback attempt returned "Unable to play media" in both the in-app
browser and Chrome. The earlier Vimeo observations above remain the evidence
from actual playback. The local GIF contact sheets were re-inspected separately;
their timestamps are not Vimeo timestamps, and their fall/reset annotations
must not be transferred to this edited video.
