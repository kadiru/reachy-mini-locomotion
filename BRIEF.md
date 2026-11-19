# Reachy Mini — Quest to Locomotion

## Priority

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
