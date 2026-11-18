# Reachy Mini — Quest to Locomotion

## Priority

Reproduce the user's previously observed motion before treating autonomous locomotion as proven.

## User context

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
