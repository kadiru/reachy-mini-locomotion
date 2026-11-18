# From Quest wobble to repeatable locomotion

## Mechanism to investigate

User-observed: rapid head swings on a Wireless Mini made the robot wobble left/right and travel on a flat surface. Working hypothesis: head inertia and shifting center of mass alternately unload parts of the foot, allowing rocking/slip; asymmetric timing, yaw phase or contact friction yields net travel. This is a hypothesis, not a verified dynamics explanation. Purely symmetric wobbling can return to the same position; measure endpoint displacement rather than head excursion or accumulated pixel motion.

The supplied GIF now visibly supports whole-body rocking coupled to head turning and tilting, and includes one topple plus a reset. Preserve both head turning and tilting as candidate components; do not reduce the recovered motion to a single roll sinusoid without checking the bridge. The frame annotations are in `GIF_OBSERVATIONS_2026-10-28.md`.

Investigate three alternatives explicitly: rocking with alternating support edges; primarily sliding; motion aided by a cable, surface slope or attachment. More than one may occur. A cable is visible in the GIF, but its role is unknown. Directional control and the user's impression of being able to move “anywhere” need multiple-heading trials before being claimed generally.

## M0 — Recover and observe

1. Locate the original robot-facing bridge using `docs/RECOVERY.md`; record original files, SDK version, hardware revision, limits, mapping and startup commands. Emcee sender is recovered; actuation mapping remains missing.
2. Record a low-amplitude axis check to establish sign/order, control flags and neutral pose. Preserve the previous successful setup when identified. Never infer deployed settings from a code default alone.
3. Use a floor-level test area, fixed external calibrated view of a rigid foot marker and a side view of contacts, with world scale in frame. Keep the surface/foot unchanged for the first reproduction. Document power/cable and any human contact.
4. Capture rest → short manual motion → rest on the common clock. Record raw Quest input, submitted targets, measured joints, available IMU, external base pose, video PTS, sync markers, cycle boundaries and falls/abort/assistance.
5. Start with one short trial. Define a cycle by a return to the same swing phase (e.g. positive-going zero-crossing of the recovered roll signal), not by arbitrarily cutting the clip every second. Record a static interval to measure tracking noise/drift.

M0 exit: one synchronized, reviewed physical trial with reproducible control configuration and positive net base displacement exceeding the measured uncertainty and rest drift. This establishes observed teleoperated locomotion for that setup, not replay or learning. Synchronization and fall review are required before promotion.

## M1 — Replay without changing the motion

The implemented CLI generates an offline schedule of submitted SDK targets with original timing. Hardware replay remains to be built after the bridge, limits and neutral start pose are known.

Before actuation, verify a start-pose transition, single active command owner, reachable target envelope, rate/acceleration limits, stale-command handling and the installed daemon's stop/hold behavior. A generic cancel-move call may not stop a continuous external set-target sender. Tracking loss in the recovered Unity sender retains buttons; the receiver therefore needs its own timeout and tracking-valid check. This is a concrete property of this bridge, not a general checklist.

Replay one clean segment at the recorded rate with operator supervision; log new command submission times, measured response and video. Slowing the segment changes inertial forces and may remove travel, so any time scaling is a separately labeled experiment. Do not rescale amplitudes upward during reproduction.

Then collect 5 independent trials × 10 cycles on the original surface, resetting start pose/heading between trials. Include all attempts, failed starts, falls and assistance; do not select only successful clips. Include rest/no-command controls and at least two headings. These are proposed experiment sizes, not completed runs.

Report per-cycle forward and lateral displacement, net travel, yaw change, speed, within-trial SD, between-trial SD, completed/attempted cycles, tracking loss, fall count/fall fraction and boundary exits. Compare teleop vs replay using the same surface, camera calibration and initial conditions. Declare repeatable travel only when independent trials consistently exceed calibration/rest noise; set the numeric target after measuring the observed baseline, not from synthetic data.

M1 exit: open-loop recorded replay reproduces direction and displacement under stated conditions. Call this **replay**, not learned autonomy.

## M2 — Imitation or small parameter search

Fit a compact phase trajectory to successful demonstrations: roll/lateral/head pitch/body-yaw components, amplitudes, offsets, relative phases and frequency. Use only components actually present in the recovered mapping. First fit reconstruction error; then compare to original replay.

- Imitation: behavior cloning predicts the next validated target from phase and measured state. Split train/validation by whole trial and surface, not adjacent frames. A fitted periodic motion remains a learned motion generator; a claim of closed-loop adaptation needs disturbance/feedback tests.
- Parameter optimization: bounded search over a few trajectory parameters within the physically demonstrated envelope, initially on CPU in a calibrated simulator. Evaluate every candidate across multiple initial states and friction settings; rerun the original replay baseline with the same conditions.

M2 exit: a candidate beats the replay baseline on held-out trials without increasing falls, assistance, thermal/load concerns or boundary exits. Distinguish an optimized script from an online policy.

## M3 — RL after model validation

No training has started. No shared Ubuntu GPU jobs were touched. Before RL, complete the free-base/contact/actuator work in `SIM_AUDIT.md` and validate against held-out physical replay. Avoid training a policy to exploit a fixed base, missing foot geometry or ideal torque limits.

Observation: joint positions/estimated velocities, previous target, phase, available IMU orientation/rates, target direction and measured/estimated base velocity. A deployable policy cannot depend on simulator ground-truth contact forces or world position unless equivalent sensing exists at deployment; use them only for privileged training/evaluation and disclose that design.

Action: bounded residuals around a validated periodic trajectory or small bounded trajectory parameters, at the measured effective control rate. Keep motor/IK/velocity constraints outside the learned policy. Start from imitation/trajectory initialization if it improves stability; do not grant a random initial policy unrestricted hardware control.

Suggested normalized reward per step:

`r = w_p * delta_forward / L - w_l * abs(delta_lateral) / L - w_h * heading_error² - w_u * excess_tilt² - w_s * ||a_t-a_(t-1)||² - w_t * tracking_error² - w_e * energy / E - w_f * fall`

`L` and `E` are declared normalization scales. Use signed progress so oscillating back and forth cannot collect positive reward twice. Apply tilt penalty above a demonstrated safe wobble envelope, not at every tilt (rocking is the mechanism). Select weights after measuring baseline magnitudes; predeclare them before evaluation. If motor torque/current is unavailable, omit physical energy or label a command-velocity proxy explicitly; never call that measured energy.

Terminate an episode on a fall, assistance, boundary exit, invalid/stale observations, infeasible target or supervisory limit. In simulation a terminal fall penalty is added once; on hardware the supervisor stops further targets and uses the verified stop behavior. Record truncated episodes so failure does not disappear from the denominator.

Evaluation: fixed held-out seed list, multiple headings, varied starts and friction/contact/inertia/latency within measured bounds, plus held-out real surfaces only after conservative supervised transfer. Compare identical-condition replay, imitation/optimization and RL. Report mean/median forward progress, between-trial uncertainty, completion rate, lateral/heading error, falls, assistance, tracking gaps and saturation. Keep policy/checkpoint/seed/config hashes in every trial. A montage alone cannot demonstrate reliability or autonomy.
