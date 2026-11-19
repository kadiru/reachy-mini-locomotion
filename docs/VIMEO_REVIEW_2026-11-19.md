# Compound swing reference — November 19, 2026

Reopened [reachy reach](https://vimeo.com/kadiru/demo-reachy-reach), resolving to
video 1243537656 / 5c824721ae. This playback is a roughly one-minute edit with
real footage on the left and our earlier simulated sway on the right. This
supersedes the previous review's duration/layout observation; the URL is the same.

Viewed the opening at displayed times approximately 0, 2, 5 and 9 seconds.
The physical robot turns and rolls its head while its base rocks; apparent
progress toward the laptop is visible. Around 5 seconds the real body is visibly
tilted while the head is turning/tilted. By around 9 seconds the video has cut
to the robot standing between yellow figures. No pixels from the simulation
inset were used as evidence about the physical robot.

The user's explanation is the working mechanism: larger compound roll and yaw,
an unloaded opposite rim, then head deceleration timed to support on one edge.
The visible motion is consistent with that account. The clip does not provide
joint commands, velocities, force measurements, metric displacement or an exact
braking/contact phase. Camera motion, partial contact occlusion and editing
prevent treating it as an instrumented imitation dataset.

Simulation changes encode the hypothesis as a bounded swing-and-hold waveform,
with separate roll/yaw sharpness and relative phase. Command and motor limits
still determine achievable braking. No instantaneous velocity reset, impulse,
base motion overwrite or artificial ground force is allowed. Ground contacts
and friction supply external reactions, so head and body yaw are not an isolated
angular-momentum-conserving pair.

New saved states include side contact normal forces, joint velocities and
actuator torques, plus the head subtree's angular momentum about its own COM.
That supports phase/contact diagnostics in simulation without asserting that
observed correlation proves the physical mechanism.
