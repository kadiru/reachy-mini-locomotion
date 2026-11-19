# From Quest demonstrations to autonomous rocking locomotion

Objective: learn repeatable forward travel from the user's head-driven,
penguin-like Reachy Mini motion, then assess transfer to the Wireless robot.
The November 18, 2026 user clarification identifies a Quest 3S and controller.

The November 19 clarification adds [two separate gait styles](GAIT_STYLES.md):
rapid small steps and larger alternating-edge steps with deliberate torso
roll/yaw. Preserve a separate policy for each; the current 57.39 mm/s nominal
simulation is the rapid style, and the large-step style remains a target.

## Choose MuJoCo first

Reachy already has an [official MuJoCo model and SDK simulator](https://huggingface.co/docs/reachy_mini/en/platforms/simulation/get_started).
The repository now has a short CPU contact experiment derived from it. Our first
uncertainty is physical fidelity: base support geometry, coupled head motion,
motor response and friction. Faster training cannot resolve those unknowns.
Consider Isaac Sim/Isaac Lab later if a validated environment needs larger
parallel experiments. No Isaac integration or shared-GPU job is started here.

## Work packages and acceptance evidence

| Stage | Deliverable | Evidence needed before advancing |
|---|---|---|
| 1. Capture | Recover/reconstruct Quest mapping; synchronize input, submitted targets, measured joints/IMU and external foot pose | One measured reproduction; stationary drift baseline; cable/surface/foot dimensions; all failures and interventions marked |
| 2. Replay | Reproduce the useful motion under the robot-local guardian | Proposed five independent trials of ten cycles; displacement exceeds tracking uncertainty; repeatability and falls reported |
| 3. Fit the simulator | Free foot, measured collision shapes, correct mass/COM, motor limits/latency and SDK frames | Replay measured targets; compare head/joint trajectories, tilt/contact phase and foot displacement; reserve entire trials for validation |
| 4. Learn a compact gait | Fit coupled head roll/pitch/yaw phase, amplitudes, bias and frequency from demonstrations; optimize parameters in simulation | Outperform recorded/replayed baseline on held-out physical parameter ranges, without exploiting initial settling, slipping-only reward or falls |
| 5. Residual RL, if useful | Small feedback corrections around the demonstrated gait | Observations available on hardware; gains beyond the compact baseline; multi-seed, held-out surface/delay/noise evaluation |
| 6. Transfer | Short supervised trials through the existing guardian | Start with bounded cycles and agreed limits on the measured setup; compare prediction vs real logs, stop on divergence, refit before expanding |

Trial counts and evaluation targets above are proposals, not completed results.
The Vimeo video supplies qualitative motion evidence. It does not contain the
aligned action/state labels needed for behavior cloning. New teleoperation
recordings are the most direct path to those labels.

## Proposed policy contract

Observe phase, commanded/measured head and body state, IMU orientation/rates,
previous action, and foot pose/velocity from a calibrated external tracker.
Use simulator-only contact forces for diagnostics, not hidden policy inputs
unless matching real sensors are added. Deploying without an external tracker
requires a separately validated state estimator.

Start with a periodic task-space trajectory passed through validated IK; add
small bounded residual head targets only after the baseline works. Do not expose
arbitrary raw Stewart joint actions before reachable workspace/closure and motor
constraints are represented. Stop/hold logic stays independent of the policy.

Evaluate net displacement in the **initial foot forward frame** per cycle and
per second, lateral drift, heading error, falls/interventions, tracking error,
energy/effort and action slew. Exclude artificial initialization motion from
reward. Terminate on numerical warnings, contact/closure anomalies or falls.
Use style-specific progress objectives with no-fall and actuator constraints.
The rapid style prioritizes speed; the large-step style must also demonstrate
larger signed cycle progress and alternating lift/support transfer. Do not apply
a blanket penalty to the useful tilt, yaw or braking that creates the step.
Separate within-cycle torso oscillation from persistent heading/path drift;
the interpretation of the 20° heading tolerance is pending user clarification.

Randomize measured uncertainty in mass/COM, friction/contact compliance, motor
strength, latency and observation noise. Hold out parameter combinations and
whole demonstration trials. Validate against zero-command, replay and simple
periodic baselines; check two timesteps and both mirrored excitation directions
to expose numerical/asymmetry artifacts before treating small travel as success.

## Immediate next action

Use [FIRST_RECORDING_SESSION.md](FIRST_RECORDING_SESSION.md) to commission the
existing bridge and collect a rest → wobble → rest recording with a fixed camera.
In parallel, measure/photo the underside and weigh the complete Wireless robot.
Those measurements replace the contact proxy; then implement task-space replay
in the moving-foot model. Physical controls and Unity sender still need hardware
validation. Compact gait optimization and heading feedback now exist in
simulation; no learned physical locomotion policy has been verified.
