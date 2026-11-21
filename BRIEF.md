# Reachy Mini — Quest to Locomotion

## Priority

For overhead balance tests, vary simultaneous ball count from one through ten
on both edges. Continue physics through the landing, recording actual non-foot
floor contact, a return to flat, and a recovered edge hold separately.
[Overhead drop results](docs/OVERHEAD_BALL_DROPS.md).

Current side-balance videos should show balls released horizontally in free
flight, striking Reachy directly from the side before touching the floor.
Check the contact order for every episode included.

For side balance, compare airborne and rolling hits across speeds on both
supporting edges. Log the actual first contact, incoming velocity and recovered
edge hold. Separate floor-first throws, head joint stops and confirmed falls.
[Results from the frozen policy](docs/SIDE_BALL_IMPACTS.md).

For the separate efficient walking policy, maximize signed forward distance
over a fixed walking time while minimizing directional head swings per minute.
The physical demonstration felt close to one swing per second and under two.
Count one leftward or rightward move as one swing. Allow rapid acceleration and
braking within a single move and evaluate whether braking coincides with
single-edge support. Keep the waist target fixed, the battery motor envelope,
and the 20° limit on heading after each complete pair.

Measure the recovery envelope in simulation. Continue through 35° of body tilt,
roll faster balls at both supporting edges, and distinguish complete recoveries,
floor contacts and joint-limit stops. Keep the zero waist target and battery
motor model. The 230 baseline ball trials, 247 fallback trials and controls, and 740 raised-start checks are documented
in [Ball speed and high-tilt catch results](docs/HIT_SPEED_AND_40_DEGREES.md).

Latest request: train noisy capture, hold and controlled return, adding actual
rolling-ball collisions from both directions during left/right edge balance.
Two nominal natural-start sequences now pass all four bounces and return flat;
the selected balls are 8 cm diameter, 10 g, released at 0.3 m/s. Recovery begins
within 0.32–1.34 s and is confirmed by at least 0.5 s of strict edge balance.
Frozen impact qualification is **7/28**, hold/return is **6/8** (front/back
artificially raised), and moving-front capture/return is **1/2**. Natural
front/back launch and broad disturbance robustness remain unresolved.
Policy: `configs/balance-recovery-experimental.json`; methods and failures:
`docs/BALL_BALANCE_2026-11-20.md`. Keep the fixed waist target, existing motor
model, all guards and earlier evidence. The selected 1 kHz gyro averaging into
50 Hz feedback differs from stock SDK processing and needs hardware matching.
Next: resolve timestep/contact sensitivity and qualify the actual IMU pipeline
before increasing impact severity or composing these holds into walking.

Latest request: extend edge balancing to the **front and back rims**. Separate
fore–aft dynamics models and pitch/rate feedback now hold both edges from
explicitly raised starts for 19.76/19.54 seconds in 20-second simulations.
Natural launch/capture from flat remains unresolved; the hold sensitivity
suite passes 9/18, with noise and mass failures. Keep the same motor limits
and fixed waist target. Policy: `configs/foreaft-balance-experimental.json`;
methods and limitations: `docs/FOREAFT_BALANCE_2026-11-20.md`. Next: improve
the capture region and launch timing, then qualify natural starts. Preserve
the previous left/right skill and walking policies.

Current request: learn a separate **sustained balance on either base edge**.
Start flat, lift one side with the head, arrest the outward fall near the
tipping equilibrium, and hold there. Keep the waist target zero and the
existing motor/IK/joint/contact limits. Flat recovery and transient roll peaks
do not count as edge balancing. Artificial tilted resets are permitted for
isolated curriculum training but must be labeled and excluded from evidence
of a natural lift-and-capture. Preserve the walking policies.

Outcome: both sides now lift naturally and hold their edge for about 57 seconds
in 60-second nominal simulations at 0.125 ms, around 31–32° roll. Selected
policy: `configs/edge-balance-experimental.json`; video and exact evidence are
in `docs/EDGE_BALANCE_2026-11-20.md`. Sensor-noise/delay checks and several
timestep checks still fail; capture robustness is the next priority. The
waist target and motor limits remain unchanged, with no hardware transfer.

November 20 outcome: 20-cycle nominal walking reaches 3.88 mm/s, 5.14% faster
than the previous 20-cycle baseline; both directions pass at both timesteps.
All 42 known single-pair and six difficult four-pair conditions pass. New
four-pair qualification is 11/16: two tilt guards and three turn/progress
failures remain. Selected policy: `configs/penguin-sustained-trained.json`.
See `docs/SUSTAINED_PENGUIN_2026-11-20.md` for direct versus equivalent-path
validation, bounded final-retry steering tolerance, and full failed evidence.
The waist target and motor limits remain unchanged; no hardware calibration
or transfer has been performed. Next priorities are the new holdout failures
and physical system identification, preserving the achieved nominal speed.

Latest outcome: faster feedback gait is 25.8% faster nominally; all 26 previously
seen cases and all four finer failure rechecks pass. Fresh validation is 14/16,
with one tilt guard and one heading/progress failure still open. Selected config:
`configs/penguin-faster-feedback-trained.json`; see STATUS and
`docs/FASTER_PENGUIN_2026-11-19.md`.

Current follow-up: retrain on the failed conditions and shorten pauses to make
the composed gait faster without falls. Keep fixed waist target, physical motor
and command-profile limits and all guards. Preserve previous policies/evidence,
use a new unseen qualification split, and record the best tested result.

Latest result: bilateral feedback recovery passes 26/26 and pivot/walking pass
20/20 original development checks. The two-pair video advances 88.6 mm, but
broader unseen checks still fail (13/16 passes for each mode). See STATUS and
`docs/BILATERAL_BALANCE_2026-11-19.md`; those failures remain the next priority.

Current request: train both sides using signed roll and roll-rate feedback so
recovery and edge pivots pass the disclosed model stress tests, then combine
them into alternating penguin steps and show a recorded simulation video.
The waist holds a constant neutral target with bounded torque; motor limits,
joint guards and the 20° full-cycle heading criterion remain in force. Train
against the previous failing conditions and reserve fresh conditions for final
qualification. A failed or incomplete check must not be counted as a pass.

Latest user direction: learn a **primitive curriculum before more walking
optimization**. First discover how pure head sway lifts an edge and produces a
sideways fall; then learn to arrest that fall and recover; then add head yaw and
braking while one edge supports the robot to learn an in-place edge pivot.
Compose those behaviors into larger penguin steps later. Do not reward forward
speed during the initial balance/pivot experiments.

The user clarified that the waist **may hold a fixed angle**. Do not command
waist rotation. Measure foot rotation on the floor separately from torso and
waist-joint rotation, and disclose holding torque/work. Preserve an unpowered
waist comparison. Deliberate falls are authorized in isolated simulation
diagnostics only; do not change the walking evaluator's 35° guard or the motor,
IK and actual joint-limit checks. A guarded invalid target is not a fall or a
successful catch. A tipping threshold depends on motion, angular velocity and
model assumptions, not only a single tilt angle.

Preserve both walking policies and all earlier failed qualification evidence.

Maintain **two distinct walking styles**, per the latest user clarification:

- Rapid left/right headshakes for fast, small steps. Preserve the current
  57.39 mm/s policy as this mode's baseline.
- Larger sway, head turn and timed stop, deliberately transferring support
  between base edges while the torso tilts and rotates to advance the opposite
  side. Develop this as a separate mode, with larger signed progress per complete
  left/right cycle and verified alternating support; speed alone must not collapse
  it into the small-step style.

The user has confirmed that **20° applies to overall heading after each full
left/right cycle**; larger intermediate torso swings are allowed. Train the
large-rocking mode using that completed-cycle constraint, measured relative to
startup without resetting the heading reference each cycle. Keep instantaneous
extrema visible as diagnostics. Motor/IK limits and the no-fall checks remain
unchanged for both modes. Preserve the rapid gait and render the best validated
large-rocking candidate. See `docs/GAIT_STYLES.md`.

Latest follow-up: push beyond the 48.19 mm/s speed-first result with larger
compound head swings and sharper braking, still upright and within 20° heading
change. Freeze both the physical motor model and the last software command
profile (10.7 rad/s, 360 rad/s², 25 Hz). Search waveform/phase/amplitude and
heading feedback, check sustained speed at finer timesteps, and show the fastest
validated result in a real-time video. Do not replace prior evidence.

Latest November 19 steering: maximize **forward speed**, keeping the simulated
robot upright and allowing up to 20 degrees of heading change. Smoothness and
lateral-drift minimization are no longer objectives. Test the user's proposed
rapid compound head swing followed by sharp braking while supported on an edge.
Distinguish rocking/rim lift from genuine flight using contact force and clearance.

Keep the current physical motor torque/speed and joint limits, mass and contact
model. Compare faster software command profiles explicitly: the old 6 rad/s,
60 rad/s² and 5 Hz target filter were conservative design choices, not measured
firmware limits. Do not relabel those choices as motor specifications or weaken
physical caps. Retain the earliest 1.2 Nm simulation as a historical reference,
not a hardware-plausible candidate. Select on longer runs and finer timesteps;
evaluate fresh model variations only after selecting the policy. Render the best
verified candidate at real-time playback, with honest displacement and heading.

The preceding heading-feedback objective and its unresolved strict stress-suite
results remain historical evidence in STATUS.md; the user has superseded that
optimization priority with speed-first movement.

Reproduce the user's previously observed motion before treating autonomous locomotion as proven.

## User context

November 19 steering: prioritize **forward progress over smoothness**, with less
turning and realistic motor bounds. User describes compound roll/yaw, edge
support, and a sudden head stop transferring reaction torque to the body.
Treat that as a mechanism to test against the physical video; preserve the
fast candidate as a development baseline, while reporting model sensitivity.


November 18 clarification: the headset was **Meta Quest 3S**. User requests a
GitHub repository and a learning/sim-to-real project based on head-driven
penguin-like motion in [reachy reach](https://vimeo.com/kadiru/demo-reachy-reach).
The user also confirmed this video was made on **battery power**; actual motor-bus
voltage under load remains unknown.
Use the recovered recording software and new MuJoCo contact probe as the starting
point; the historical description below predates this model-name correction.

The user has already used Meta Quest 3 goggles and a controller to manually make Reachy Mini move/walk quite a bit. Preserve that observation and investigate the mechanism; do not dismiss it simply because the product is normally stationary. They want a compilation of people saying Reachy Mini cannot walk, using LinkedIn, X, YouTube or other originals, followed by a locomotion demo and ideally reinforcement learning. The specific Reachy version, teleop bridge and code locations are unknown.

## Initial milestone

Develop the first milestone around recovering the existing Quest-control approach, recording synchronized commands/measured states/video and quantifying displacement, then replaying the motion. Ask one compact question for the existing teleop code or previous recording and Reachy version; continue official SDK/simulator research independently. Create a minimal local recording schema and analysis/scaffold for comparing displacement per cycle, repeatability and falls; use synthetic examples only when clearly labeled and never report them as robot evidence. Inspect official MuJoCo support and identify whether a free-moving base, contact geometry, friction and actuator limits support the observed mechanism; do not assume the stock simulator is ready for locomotion RL. Propose a staged path from repeatable replay to imitation or parameter optimization to RL, with reward and evaluation definitions. In parallel search public sources for original 'cannot walk' videos/posts. Keep a source ledger with URL, author, exact quote, timestamp when actually verified, and verification status. An article or search snippet is a lead, not a verified video clip. Do not download/repost clips or message creators. Prepare a short reveal-video shot list, and clearly distinguish teleoperation, replay and learned autonomy. Do not start GPU training in this initial assignment.

## Starting sources

- https://huggingface.co/docs/reachy_mini/en/index
- https://huggingface.co/docs/reachy_mini/en/platforms/simulation/get_started
- https://github.com/pollen-robotics/reachy_mini

## Coordination

Work directly on this project and make concrete progress in your initial turn. Read AGENTS.md, BRIEF.md and STATUS.md in your assigned folder first. Own only this folder; sibling projects and portfolio files belong to other chats. Use project-local environments and preserve existing user work. Research and code can proceed without hardware; distinguish implemented, mocked, simulated, and physically verified results. Update STATUS.md with evidence, exact runnable commands, open questions and the next action. Keep user updates concise and useful. Ask missing-information questions asynchronously where possible while continuing independent work. Do not create more chats or recurring automations, publish posts, contact other people, buy resources, or start paid/long training jobs as part of this initial assignment. Existing humanoid training has priority on the shared Ubuntu GPUs; do not alter those jobs. The coordinator will read your status; no message back is needed.
