# Reachy Mini — Quest to Locomotion — Status

## November 19, 2026 — personal clones resynchronized without losing work

User confirmed the scope is this Mac and `kfuUbuntu`, with Nimbus/EY excluded.
Resynchronized 11 personal Git databases / 15 worktrees: six databases on the Mac
and five on Ubuntu, including four found in Ubuntu's mounted older home directory.
Six branches advanced by fast-forward, with no new merge commits or history
rewrite. Two Sircus detached review checkouts keep their corrected pinned commits.
The local-only Neuralink visualization branch and its work remain unchanged;
its inactive master branch now matches the current remote.

Verified all 24 original dirty files byte-for-byte and compared staged and
unstaged binary patches before and after. This includes the dirty Ubuntu Emcee
Android files while its main branch advanced six commits in disjoint paths.
All active local branches, tags and remote-tracking histories are free of the
retired personal attribution; no stashes were present. Each clone has a verified
bundle, prior config/index, patches and dirty-file copies in
`.git/identity-repair/20261119-clone-resync/`.

Mac identity guards remain active. The five Ubuntu clones now have explicit
personal identity settings and repository-scoped guards that preserve existing
hooks; author and committer override rejection was tested without making commits.
Ubuntu has no GitHub CLI login or configured GitHub SSH trust/authentication, so
minimal verified personal-history bundles were transferred from the Mac instead.
Its guards stop future pushes until authentication can be verified as `kadiru`.
Shared authentication settings and Nimbus/EY configs/code were left untouched.
No training job, container or physical robot runtime was changed.

Report: `experiments/reports/personal-clone-resync-2026-11-19.json`.
Projects without remotes retained their existing work; none was newly published.


## November 19, 2026 — personal repository identity audit completed

Audited all 31 `kadiru`-owned GitHub repositories plus three other accessible
repositories: 221 branches and 34 tags (including a concurrently published
Roboturk branch, separately checked and clean). The additional affected GitHub repository
was Sircus: five incorrect author/committer identities corrected, with 424 commit
hashes and all 162 branch tips changing. Published atomically with exact leases;
all published refs and GitHub attribution for the five repaired commits verified.
Three signatures on rehashed commits were removed because they could no longer
verify the changed objects. Original signed objects remain in recovery bundles.

Also corrected 17 local Roboturk commits and one local personal-portfolio commit;
these repositories have no remotes, so nothing was uploaded from them. Code,
messages, timestamps, other contributors, and working files/indexes were preserved.
Roboturk acquired concurrent uncommitted work during the audit; the first apply
stopped, that work was separately backed up, and the metadata-only repair preserved
it. All 21 non-Nimbus local worktrees now have clean active attribution and the
correct personal author/committer identity. Existing project hooks are retained
behind the global identity dispatcher. Guard tests cover new-repository creation,
identity overrides, hook delegation, push input, and old-history rejection.

User clarification: `kadiru` and `KineticUtopia` are distinct users. Nimbus/EY is
excluded from this cleanup; never upload EY code to personal GitHub. No Nimbus
history or code was changed or uploaded. Ten Nimbus local configs touched during
the initial broad audit were restored exactly. Scope and account separation are
saved in global Codex instructions; excluded checkouts reject personal pushes.

Report: `experiments/reports/personal-git-identity-audit-2026-11-19.json`.
Private bundles, mappings and worktree snapshots remain in each repaired repo's
`.git/identity-repair/20261119-other-repositories/`. Hashes changed; other clones
must resynchronize with the corrected remote history without merging old commits.
The earlier Reachy/humanoid correction below remains in place.


## November 19, 2026 — corrected GitHub attribution to kadiru

User requires all future authoring/committing/pushing through `kadiru`, never
`KineticUtopia`. The incorrect global Git identity was corrected to `Kadir Uyanik
<717445+kadiru@users.noreply.github.com>`, with explicit local identity settings,
`user.useConfigOnly`, and GitHub authentication through the active `kadiru` account.
The preference is also saved in the global Codex instructions. Both requested
repositories have pre-commit/pre-push guards that reject the retired identity;
pushes additionally require GitHub authentication as `kadiru`.

Rewrote the eight prior Reachy commits, preserving their trees, messages, dates
and topology. Latest replacement: `1623eca9a055ff6d44b31a86cebd8cdc219a594b`.
The user also explicitly requested correction of the humanoid repository shown
in their screenshot: 53 misattributed main-branch commits plus 773 on its saved-
history branch were corrected; its existing correct author/committer metadata was
preserved. Its working files and staged contents were verified unchanged.
No robot, GPU job, or project implementation was changed for this repair.

Published with exact force-with-lease checks (the two humanoid branches together
atomically). GitHub API verification confirms all published authors/committers
resolve to `kadiru`. Verified local bundles, prior configs, dirty-file snapshots,
old/new commit mappings and verification receipts are retained privately under
`.git/identity-repair/20261119-033149/` in each repository. Commit hashes changed;
other clones must resynchronize with the rewritten history rather than merging
the old, misattributed history back. Identity guards were tested for normal
commits, explicit author overrides, and attempted pushes of bad-identity history.


## November 19, 2026 — heading feedback trained; all-pass stress target unmet

Added bounded head-roll/yaw/timing corrections driven by a simulated heading
estimate (60 ms delay, 0.25° noise), with no privileged model/contact inputs.
Body yaw remains zero; existing mass, motor torque/speed/acceleration/latency
models and pose/IK limits are unchanged. Optional estimator, amplitude and
harmonic steering variants were investigated but are disabled in the selected
configuration. Hardware remains untouched.

**Selected nominal 1 ms run:** 324.7 mm in 20.52 s, 15.83 mm/s, **89.2%** of the
previous fast gait's speed; -16.5 mm lateral, -2.54° net turn, 5.89° maximum
measured excursion (5.62° maximum error from startup). Peak head torque 0.3982 Nm
under 0.416 Nm cap; actual joint-angle margin >=0.2425 rad. Nominal 2 ms speed
16.82 mm/s. Video uses exact saved evaluated states at 1x playback.

**Original stress cases, >=12 s:** still **0/12 pass** the original strict criteria.
Mean forward speed improves 12.08 ->18.62 mm/s; minimum 4.29 ->13.90 mm/s; worst
heading excursion 110.07° ->24.03°. At >=20 s: 0/12 pass, minimum 13.78 mm/s,
worst turn 26.30°. Same gait/startup with feedback disabled gives worst turn
81.60° and mean speed 13.75 mm/s, so feedback provides a measured improvement.
Do not call this an all-pass or hardware-transfer success.

**Fresh seed 2026111907**, ten models x two timesteps, never used for selection:
4/20 passes versus original fast 2/20; all numerically valid and forward-moving.
Worst heading falls 118.64° ->34.60°, mean speed 11.88 ->14.62 mm/s. Fresh minimum
speed 2.11 mm/s; only 8/10 model pairs pass the coarse timestep displacement
check. One differs 7.71 vs20.87 mm/s. Ideal/noisy/delayed sensor audits still fail
stress heading constraints, including with ideal sensing.

593 recorded candidate evaluations /4,151 completed training rollouts, plus
112 steering probes. A feasibility-first search was stopped after 72 candidates
because it favored near-stationary policies; that receipt labels the interruption.
Final selection uses only >=20 s development tests, requiring >=14 mm/s nominal
at both timesteps and >=8 mm/s in all original stress cases; two stalling
candidates were rejected. Final development physics replay matched exactly.
At most four local CPU training workers; no shared GPU or robot was accessed.

**Validation:** 75 tests pass in
`experiments/reports/repository-tests-heading-2026-11-19.txt`. New tests cover
heading wrap/delay/noise, anti-windup, bounded corrections, optional phase
estimation/adaptation, and motor-bounded repeatable recorded rollouts.

Frozen configs: `configs/sway-heading-trained.json`, `configs/heading-trained.json`,
`configs/heading-sensor-assumed.json`. Final evaluation and video receipts:
`experiments/reports/motor-training-heading-final-2026-11-19.json`,
`experiments/reports/motor-training-heading-demo-2026-11-19.json`.
Videos: `media/simulation/heading-feedback-2026-11-19.mp4` (nominal) and
`media/simulation/heading-feedback-stress-2026-11-19.mp4` (predeclared stress1;
311.7 mm/20.52 s, but 24.03° heading failure). Both are 1x, fixed views, 5 cm grid.
Full protocol, limits and receipts: `docs/HEADING_FEEDBACK_2026-11-19.md`.

```sh
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m reachy_locomotion.sway --config configs/motors-datasheet.json --gait configs/sway-heading-trained.json --feedback configs/heading-trained.json --sensor configs/heading-sensor-assumed.json --cycles 38 --warmup-cycles 3 --output experiments/generated/heading-next.json
.venv/bin/python experiments/render_sway.py experiments/generated/heading-next.json --output experiments/generated/heading-next.mp4 --slowdown 1 --title 'HEADING FEEDBACK' --diagnostics --frame-path
```

The CLI example uses 2 ms; the shown videos override the model to 1 ms. Next:
the simultaneous fast/all-pass objective remains open. Inspect within-stride yaw,
steering oscillation and lateral drift together, consider richer phase-dependent
control using observable joint state, and calibrate battery-powered servo/contact
response before physical transfer. The existing hardware commissioning guard is
unchanged; no physical-ready flags were set.

## November 19, 2026 — progress-first compound swing and best-result video

User requested faster progress, less turning, published motor bounds, a best-run
video, and a compound roll/yaw stop timed to edge support. Rewatched Vimeo's
opening 0–10 s. The current edit includes real footage plus the earlier simulation
inset; only the real footage was treated as physical evidence. Review is in
`docs/VIMEO_REVIEW_2026-11-19.md`.

Extended `Gait` with bounded roll/yaw swing-and-hold shaping and small biases;
added maximum unwrapped heading/lateral excursion metrics and saved side-force,
velocity, torque and head-subtree angular-momentum diagnostics. Motor model,
0.416 Nm nominal head cap, 1.475 kg mass and contact geometry are unchanged.
Targets remain rate-limited and IK-checked; no external pushes or base writes.

`train_progress.py`: 624 candidate evaluations / 1,200 CPU training rollouts
(12×36 nominal exploration; 8×24 four-domain conservative refinement). Objective
rewards speed without a smoothness penalty, subject to heading/drift/tilt checks.
78 invalid IK outcomes retained. `select_progress.py` independently selects the
fastest nominal candidate from exploration only, with 12 s tests at 1/2 ms and
20 s tests of eligible candidates, before fresh stress tests. All training uses
at most four local CPU workers; no robot or shared GPU was used.

**Selected nominal result:** 363.9 mm forward, -27.6 mm lateral in 20.52 s at
1 ms; -7.76° net measured turn, 9.03° maximum excursion after startup (11.52°
including startup). 2 ms result 371.2 mm, a 2.0% displacement difference. Previous
smooth gait gives 50.9 mm over matched 20.52 s: **7.15× speed improvement**.
Peak head torque 0.4055 Nm under 0.416 Nm cap; head-joint speed <=6.02 rad/s;
no 99%-cap saturation; actual joint positions have >=0.219 rad limit margin.
Foot tilt 3.54°, rim lift 5.90 mm. Actual head yaw roughly -50° to +54°.

**Robustness limit:** selected fast gait advances in 12/12 fresh stress cases,
but 0/12 meet full turning/drift criteria; maximum turn spans 12.6–110°. Previous
smooth gait passes 10/12 on the same set. Do not represent this as robust transfer.
The separate conservative branch reaches 6.1–6.6 mm/s nominal; it has a different
stress set, so its pass count is not directly comparable.

Mechanism: 76.1% sampled single-edge support, all sampled yaw slowing on an edge.
Removing yaw or shifting yaw phase 90° cuts nominal speed to about 4 mm/s; removing
roll plus lateral translation nearly eliminates motion. Removing dwell shaping
retains about 17.3 mm/s: coupled motion/timing matters more than aggressive dwell.
These are simulated diagnostics, not physical force identification.

Video: `media/simulation/progress-fast-2026-11-19.mp4`, 22.16 s / 554 frames at
25 fps, real-time playback, fixed front/side cameras, 5 cm grid, displacement,
heading, support and head-slowing overlays. Inspected preview and near-end frame.
Saved states: `experiments/generated/progress-fast-20s-2026-11-19.json` (ignored).
Reports: `experiments/reports/motor-training-progress-summary-2026-11-19.json`,
complete search/selection/demo/stroke receipts. Frozen policies:
`configs/sway-progress-fast.json`, `configs/sway-progress-conservative.json`.
Full explanation: `docs/PROGRESS_TRAINING_2026-11-19.md`.

**Validation:** 67 local tests pass; log
`experiments/reports/repository-tests-progress-2026-11-19.txt`. New tests cover
bounded periodic strokes, startup/bias limits, rejection of full spins, and
recording that does not alter physics. Additional development/selection/stress/
ablation checks total 264 rollouts, plus three saved-state demonstrations.

```sh
.venv/bin/python experiments/train_progress.py --output experiments/reports/motor-training-progress-next.json
.venv/bin/python experiments/select_progress.py experiments/reports/motor-training-progress-next.json --output experiments/reports/motor-training-progress-fast-next.json
.venv/bin/python -m reachy_locomotion.sway --config configs/motors-datasheet.json --gait configs/sway-progress-fast.json --cycles 38 --output experiments/generated/progress-next.json
```

The last CLI command uses its documented two-cycle warmup and config's 2 ms;
the shown video uses three warmup cycles and 1 ms, as specified in the report.
Next: identify battery-powered motor response/contact properties and add bounded
heading feedback/phase adaptation to retain speed across model variations.
Do not deploy this open-loop candidate to hardware based solely on its nominal
video; the existing commissioning guard and measured-state workflow still apply.

## November 19, 2026 — clarified speed regression with matched-duration runs

User pointed out that the earlier demo moved forward much better. Confirmed:
the later result improved heading/acceleration while giving up forward speed;
it should not have been framed as an overall locomotion improvement.

Ran 54 diagnostic simulations (three frozen gaits, nine model variants, two
timesteps), all with exactly 6.6 s measurement windows. At 2 ms: original fast
gait/model = 158.4 mm; changing only its cap from 1.2 to 0.6 Nm = 2.16 mm. Under
the same current 5 V model, first trained gait = 68.5 mm forward but 50.5 mm
lateral and 57° turn; smooth gait = 17.7 mm forward, 1.4 mm lateral, 0.3° turn.
The original fast result at 1 ms is 111.4 mm, showing significant timestep
sensitivity. Torque sensitivity in the old model does not prove the real
mechanism needs that torque; its passive losses/inertia were also incorrect.

Additional sensitivity: lower CAD mass boosts newer-model progress markedly;
full momentary torque bounds or looser command limits do not recover the old
fast gait. These diagnostics do not change the production config or policy.
Mass/COM, servo phase lag and contacts remain uncalibrated. User priority is
forward progress; next training should maximize speed with explicit steering/
stability constraints, keeping smoothness as a bound or secondary objective.

Artifacts: `experiments/compare_forward_progress.py`,
`experiments/reports/motor-training-forward-comparison-2026-11-19.json`,
`docs/FORWARD_PROGRESS_COMPARISON_2026-11-19.md`. Verified all 54 windows completed
at exactly 6.6 s, no external forces, and historical 158.4 mm baseline reproduced.
An initial attempt stopped at a rounded-mass guard; rerun uses the exact XML mass
sum. No production physics code changed; no new training or hardware work.

```sh
.venv/bin/python experiments/compare_forward_progress.py --output experiments/reports/motor-training-forward-comparison-next.json
```

## November 18, 2026 — motor-bounded training and robustness refinement

User requested smoother, more reliable forward motion and real motor limits;
they confirmed the Vimeo walking demonstration ran on **battery power**.
Checked official Pollen and ROBOTIS sources. Head motors are XL330-M288-T:
0.52 Nm / 103 rpm at 5 V, 0.60 Nm / 123 rpm at 6 V (stall torque / no-load speed).
The previous assumed 1.2 Nm cap was unrealistic for these published conditions.
Body uses custom XC330-M288-PG; antennas XL330-M077-T. Actual motor-bus voltage,
installed registers, continuous duty and PG gear envelope remain unknown.

Implemented `motors.py` and `configs/motors-datasheet.json`: per-model voltage
interpolation, approximate PWM/DC torque-speed response, peak torque and PWM
caps, 50 Hz target latency, speed/acceleration smoothing, actual joint-angle
bounds, torque/speed diagnostics. Corrected model mass from 1.17652824 to 1.475 kg;
tested torso and uniform mass placement. Removed inherited motor rotor inertia
from passive ball joints and exposed their uncertain friction. All inferred
parameters and source links are in `docs/MOTOR_LIMITS.md`. No hardware accessed.
CLI now defaults to this model; historical configs remain reproducible.

Ran **528 candidate evaluations / 1,872 training rollouts**, 98 control/evaluation
rollouts, and six constant-voltage battery sensitivity rollouts on local CPU
(max four workers). First CEM study: 10×24 candidates, three domains. Refinement:
12×24 candidates, four domains including 1/2 ms physics, stronger worst-domain,
heading, sideways and acceleration penalties. Refined policy found in its first
generation; later generations did not improve it. This is learned parameter
search, not RL. All 66 invalid training outcomes are retained. Initial model and
study committed as `bf18fb5`; complete receipts under `experiments/reports/`.

Refined nominal gait: **1.852 Hz**, **32.1 mm forward in 12.96 s**, -5.1 mm lateral,
-3.49° heading at 1 ms; 34.0 mm forward at 2 ms. Sampled RMS base acceleration
0.419 m/s² versus 1.172 for first trained candidate (~64% lower), no torque-cap
saturation, peak head torque 0.288 Nm under a 0.416 Nm study cap. Alternating
edge unloading ~12% per side, peak foot tilt 0.81°, rim clearance ~1.2 mm.
Large head target phase lag (~88° maximum orientation error) remains; measured
head roll/yaw stay roughly within ±36°. This is a subtle rocking shuffle.

Fresh heldouts: six uncertain parameter sets × two timesteps; **12/12 advance
more than 1 mm**, **8/12 pass** speed/lateral/heading/tilt checks versus 2/12 for
first trained policy. Four fail the <=10° heading criterion (11–15° turn).
No heldout numerical/contact validity failure. Speeds 1.72–6.90 mm/s. All seven
refined domain pairs pass a 20% +1 mm displacement timestep diagnostic, not a
formal convergence proof. After battery confirmation, nominal 5/5.5/6 V at
1/2 ms all pass (32–48 mm forward); these are hypothetical constant bus voltages,
not measurements or sag/thermal simulations. No extrapolation above 6 V.

Artifacts: `configs/sway-trained.json`, `docs/MOTOR_TRAINING_2026-11-18.md`,
`experiments/reports/motor-training-summary-final-2026-11-18.json`, complete
initial/refined/battery receipts, and
`media/simulation/motors-trained-2026-11-18.mp4` (14.08 s, real-time playback,
actual saved 1 ms states, front/side fixed cameras). Preview PNG inspected;
ffprobe verified 352 frames at 25 fps. Full states are local/ignored at
`experiments/generated/motors-trained-1ms-2026-11-18.json`.

Validation: **63 tests passed locally**, including torque-speed/braking,
voltage-extrapolation rejection, delay, per-motor torque/target-rate bounds,
mass/passive inertia, reproducible rollout, stationary control, and criteria
that distinguish numerical validity from walking success. Log:
`experiments/reports/repository-tests-motors-final-2026-11-18.txt`.

```sh
.venv/bin/python -m reachy_locomotion.sway --config configs/motors-datasheet.json --gait configs/sway-trained.json --cycles 24 --output experiments/generated/trained-next.json
.venv/bin/python experiments/train_sway.py --output experiments/reports/motor-training-next.json
.venv/bin/python experiments/refine_sway.py --output experiments/reports/motor-training-refined-next.json
.venv/bin/python experiments/battery_sensitivity.py --output experiments/reports/motor-training-battery-next.json
```

Next: read-only battery-powered voltage/register/state capture, fit actuator
phase lag and passive/contact losses, then add heading feedback and evaluate on
fresh domains. Transfer remains gated by the existing hardware commissioning
workflow. Motor specifications are sourced; dynamics/contact/thermal behavior
are not physically calibrated. No robot, shared Ubuntu GPU, or paid job used.

## November 18, 2026 — head sway produces simulated forward motion

User requested simulation-first head sway inspired by their video. Implemented
`src/reachy_locomotion/sway.py`: official Rust IK, moving-foot-relative poses,
50 Hz servo targets, smooth startup, cycle-aligned measurements excluding two
warm-up cycles, per-physics-step warning/tilt/closure/contact checks, foot force
diagnostics and saved states. No base-position writes or external pushes occur
during rollouts. Body yaw target remains zero; six head actuators cause motion.

The first 75 mm contact proxy was wider than the CAD underside. New explicit
config `configs/sway-cad-proxy.json` uses a 56 mm cylindrical approximation
informed by CAD bottom bounds and an **unmeasured 1.2 Nm cap**, replacing the
prior unmeasured 0.6 Nm hypothesis only for this experiment. Neither is a verified
hardware actuator envelope. The original contact config remains unchanged.

`experiments/sway_study.py` ran **170 CPU simulations**: 135 search cases, 18
sensitivity cases and 17 controls/extended trials; all four invalid outcomes are
preserved in `experiments/reports/sway-study-2026-11-18.json`.

Candidate requests roll ±40 degrees and yaw ±30 degrees, yaw phase +90 degrees,
frequency 4.545 Hz (rounded to 11 samples/cycle). Twelve measured cycles advance
61.1 mm at 2 ms and 47.8 mm at 1 ms, in 2.64 s. Reversing yaw phase produces
approximately -27 mm at both timesteps. Zero-motion drift is below 1 micrometre.
A 30-cycle run travels 158.4 mm forward and 6.3 mm laterally in 6.60 s; peak foot
tilt 6.43 degrees, maximum closure error 0.097 mm, alternating side unloading,
and about 10.9 mm maximum opposite-rim clearance. No numerical warning/fall limit
or non-foot floor contact occurs in that accepted run.

Important limits: motors are strongly saturated; actual head roll/yaw are much
smaller than requested (roughly ±8 / ±11 degrees). The 0.6 Nm hypothesis barely
rocks. Low friction causes large sideways drift; heading changes with timestep.
A faster 2.4 Nm candidate fails at 1 ms despite appearing stable at 2 ms.
This supports an exploratory rocking/slip mechanism, not calibrated transfer,
reliable steering or learned autonomy. No hardware/GPU training was performed.

Artifacts: `media/simulation/sway-forward-2026-11-18.mp4` (half-speed, fixed front
and side cameras, 5 cm grid, metrics), corresponding PNG/GIF, full local states
in `experiments/generated/sway-forward-2026-11-18.json`. Video is rendered from
actual saved simulator states. `docs/SWAY_EXPERIMENT_2026-11-18.md` explains
results, assumptions, failures and exact commands.

Validation: **55 local tests passed**, including static IK/frame agreement,
unreachable-pose rejection, stationary baseline, and alternating contact with
intact closed-chain constraints. Log:
`experiments/reports/repository-tests-sway-2026-11-18.txt`.

```sh
.venv/bin/python -m reachy_locomotion.sway --config configs/sway-cad-proxy.json --gait configs/sway-candidate.json --cycles 30 --output experiments/generated/my-sway.json
.venv/bin/python experiments/sway_study.py --stage controls --output experiments/reports/sway-controls-next.json
```

Next: fit motor speed/torque/latency and actual underside geometry, then reduce
saturation and sideways drift with a slower reachable trajectory. Compare to
instrumented Quest demonstrations before any physical transfer.

## November 18, 2026 — repository launch and moving-foot experiment

Current request: inspect the user's Vimeo movement, create a GitHub repository,
and begin learning head-driven penguin-like locomotion with sim-to-real intent.
Latest user correction: **Meta Quest 3S**, not the earlier Quest 3 shorthand.

Repository created: **https://github.com/kadiru/reachy-mini-locomotion** (private).
Local root is this project folder; upstream clones, raw media, real recordings,
calibration and credentials are excluded. Package installation and both CLI
entry points were verified locally. The project virtual environment's packaging
tools were upgraded to support editable `pyproject.toml` installs.

Initial implementation pushed as `c5341b9`. [GitHub CPU CI](https://github.com/kadiru/reachy-mini-locomotion/actions/runs/44390624399)
passed on Linux/Python 3.11, including package installation, a fresh pinned model
checkout/LFS hydration and the test suite. Local Python 3.9 package installation,
both CLI entry points and `pip check` also passed. Local working tree was clean
after the implementation push; this documentation update records the CI receipt.

Reviewed the opening tabletop sequence in `https://vimeo.com/kadiru/demo-reachy-reach`
through the Vimeo player. Head turning/tilting, body rocking and apparent
progress toward the laptop are visible. No metric speed or causal contact-force
claim is supported by that uncalibrated footage. Detailed observations and
coarse timestamps: `docs/VIMEO_OBSERVATIONS_2026-11-18.md`.

New implementation: installable package/CLI; `contact_probe.py`; explicit
uncalibrated contact config; CPU test workflow; four additional tests; learning
roadmap. The derived model preserves the pinned upstream XML, all original
inertias and five closure constraints, adds one free foot joint and a cylinder
contact proxy, and uses configurable unmeasured actuator limits. This supersedes
the historical statement below that no free-foot experiment exists.

Validation: **50 tests passed** locally (all physics tests executed, no skips).
Two-second hold and external-push trials ran at both 2 ms and 1 ms timesteps with
no numerical warnings. Hold drift below 1 micrometre; 2 ms maximum closure error
approximately 1.85 micrometres. The 2 N external push caused about 0.198 mm lateral
travel at either timestep. It is an artificial perturbation, not a gait. Reports:
`experiments/reports/contact-probe-2026-11-18.json`,
`experiments/reports/contact-probe-1ms-2026-11-18.json`, and
`experiments/reports/repository-tests-2026-11-18.txt`.

Commands from this directory:

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion.contact_probe --output experiments/reports/contact-probe-next.json
```

No trained policy, head-actuated simulated gait, new physical trial, robot
actuation or GPU training has occurred. The proxy is not calibrated to the
Wireless underside, battery/COM, surface or motors. Next: measure the underside
and mass; commission the existing recorder; capture synchronized Quest targets,
joints/IMU and external foot pose; implement validated SDK-frame replay in the
free-foot model. `docs/LEARNING_ROADMAP.md` describes the staged experiment.

## Historical status (retained for continuity)

Updated: October 31, 2026 (America/Los_Angeles).
State: First-session software implemented and tested with synthetic camera frames and a mock robot: authenticated Quest bridge, integrated recorder, 3D camera tracking/calibration and robot-local stop guardian. Instrumented physical reproduction and Unity/robot deployment remain pending. No GPU training, real robot actuation, publication or external messaging performed.

## Carrier concept research — Astro versus temi

User requested an investigation into programmable Astro/temi as a carrier and character partner for Mini. Research only; no hardware contacted or modified. Full source ledger and proposed checks: `experiments/reports/astro-temi-interface-research-2026-10-31.json`.

Amazon's [Alexa Web API](https://developer.amazon.com/en-US/docs/alexa/web-api-for-games/alexa-games-about.html) explicitly supports Astro, so saying it has no developer interface would be incorrect. Custom HTML/JavaScript visuals/audio and the [Smart-Motion extension](https://developer.amazon.com/en-US/docs/alexa/web-api-for-games/alexa-games-extensions-smartmotion.html) offer a legitimate limited character-app route. Actual capabilities/choreos must be queried on the device. No public arbitrary driving/velocity or neck-angle SDK, or reproducible Astro rooting procedure, was found. The university SDK announcement does not establish public access. A 2023 Home Assistant owner report describes named-location commands/TTS/routine gestures via unofficial Alexa Media Player. Source inspection of `silas-cole/claude-astro-brain-rider` confirms its control script speaks commands through TTS; its success is not a robot acknowledgment. These paths remain untested on the user's Astro, including custom-face/navigation coexistence.

For direct driving plus expressive UI, recommend temi conditionally: public Android SDK has movement, navigation, screen tilt, speech and callbacks. Current SDK contributor guidance and official REST documentation require PRO entitlement; verify user's installed model/software/subscription first. REST is for higher-level navigation/speech/call sequences. Astro cargo limit is 4.4 lb in its bin; temi's nominal tray limit is 3 kg and assembly-dependent. Mount plus Mini mass/clearance/stability remains unmeasured. No carrier integration was implemented. This exploratory branch does not replace or validate the existing Mini locomotion work.

## Hands-on session — read-only preparation started October 28

User requested the first recording session, beginning with read-only checks and camera calibration. Local checks identify this computer as “Kadir’s Mac mini”; active interface `en0` currently has LAN IPv4 `192.168.68.53` (recheck after a network change). The project environment imports Python 3.9.6, OpenCV 4.12.0 and NumPy 2.0.2 successfully. System-level macOS enumeration finds a USB camera, vendor 1133/product 2081, plus two Continuity Camera entries. The initial sandboxed camera query returned no entries; that was not evidence of an absent camera. No camera frames have been captured.

AVFoundation enumeration reports the USB camera at index 0 and the two Continuity Cameras at indices 1 and 2. Confirm the actual USB view in the calibration preview before saving images. The real Mini address/dashboard URL and camera choice have been requested. No robot endpoint has been contacted, motors commanded, calibration measurements invented, or hardware-verification flags changed. Read-only network/device queries succeeded after sandbox escalation; no approval rejection remains outstanding. Next: obtain the robot address, run `inspect-robot`, identify the USB camera in the preview, capture varied views of the ruler-checked printed checkerboard, then calibrate the fixed world/foot marker geometry. Quest preview and physical control commissioning follow those checks.

## Latest work — first-session software built

User authorized implementation while preparing the hardware. Added `profile.py`, `geometry.py`, `protocol.py`, `robot_io.py`, `guardian.py`, `camera.py`, `session.py`, `first_session.py`, `smoke.py`; direct-LAN Quest sender source `integrations/unity/ReachyRecordingSender.cs`; an explicitly synthetic preview profile; pinned host capture dependencies; printable markers/checkerboard PDFs; and `docs/FIRST_RECORDING_SESSION.md` with exact operator commands.

The new mapping is reconstructed clutched head roll/pitch/yaw plus configurable body yaw, with translation locked and antennas untouched. It is not the original deployed Emcee mapping. UDP packets use HMAC authentication, source sequence/time, tracking validity and receiver challenges. Legacy/invalid/reordered/expired packets cannot actuate. B+trigger is the deadman; grip, release, keyboard, camera loss and timeouts stop and latch. Hardware mode refuses the unverified example profile and synthetic camera calibration/input.

The guardian runs on Mini and writes only to its localhost daemon. It checks installed SDK version, analytical IK, configured joint/pose/rate bounds, tracking error, existing moves and unexpected target changes. A per-trial lease expires without fresh commands; an OS lock prevents a second instance of this guardian for the same local daemon. Other SDK applications must still release the motors. Stop requests submit the measured pose once and revoke the lease; neither torque disabling nor return-to-neutral is automatic. Physical settling/stop latency and daemon-failure behavior are unverified, and analytical IK is not a general mesh collision checker.

Camera calibration uses checkerboard intrinsics and measured marker-to-foot mounting; tracking estimates both world and foot-tag pose in 3D, rejecting small/clipped/ambiguous/missing tags. Original frames are stored before UI annotations with MJPEG compression; saved frame count is verified by decoding. Shared-host monotonic receipt times, source timestamps and robot/Mac clock-offset exchanges are recorded. Derived alignment preserves raw receipt timestamps. Camera exposure and Unity acquisition clocks are NOT hardware-synchronized; no precise exposure-phase claim is made.

**Verification:** 46 unit tests pass (`experiments/reports/first-session-tests.txt`), including prior evidence tests plus authentication/replay protection, release-to-arm, stale data, rate/IK/profile rejection, competing-writer detection, stop holding measured pose, logging failure, calibration and tracking geometry. Full synthetic pipeline passes in `data/synthetic/first-session-smoke-04/report.json`: 40/40 rendered video frames tracked, 33 mock target submissions, robot-local watchdog ACTIVE → STOPPED after input removal. Maximum position error against generated geometry was 0.0044603 m; this is synthetic camera-test error, not physical accuracy or robot travel. The threaded CLI check (`experiments/reports/first-session-cli.json`) confirms 40 saved frames decode, no unarmed commands, camera EOF detection and zero remaining workers at shutdown.

The initial synthetic test exposed ambiguous small-marker poses; the tracker now requires adequate image area and rejects ambiguous solutions, and setup instructions specify a clearer oblique view. Failed developmental reports remain labeled synthetic. OpenCV 4.12.0.88 was installed only in the project `.venv`; NumPy 2.0.2 and Pillow 11.3.0 are pinned in `experiments/requirements-capture.txt`. Unity/C# tooling was unavailable; the sender has not been compiled or deployed to Quest. No camera, Agora channel, robot or Ubuntu job was accessed.

Reproduce from `/Users/kadir/linkedin/projects/reachy-mini-locomotion`:

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion session-smoke --output data/synthetic/new-session-check
PYTHONPATH=src .venv/bin/python experiments/check_session_cli.py
```

First hands-on action: read-only `inspect-robot`, compile/install the new Unity sender in a working copy of Emcee, preview with no actuation, calibrate/verify the fixed camera, and establish the actual hardware profile and stop commissioning procedure. Then run the verified guardian on Mini and an execute session on the Mac. The user is getting hardware ready; no further repeated request for the missing original bridge location is needed.

## Prior planning — complete training and voice runbook

User requested a simple but complete guide to the programs, training tasks and hardware steps needed for “Mini, walk forward,” building on their reported Mac mini/Ubuntu local-SLM conversation system with Temi. `docs/TRAINING_AND_VOICE_RUNBOOK.md` covers hardware preparation; synchronized capture; supervised replay; trajectory fitting; moving-foot/contact/actuator modeling; residual-policy inputs/actions/rewards; proposed staged CPU PPO experiments; held-out evaluation; physical transfer; program responsibilities; and speech skill integration.

At that planning stage, all new trial counts, acceptance targets, residual scales and hyperparameters were explicitly proposals; no new runtime/tracker was yet implemented. The first-session implementation above supersedes the capture-program status. Locomotion environments, trained weights and speech integration remain unimplemented. Initial deployed-policy design depends on a calibrated live external camera; unrestricted walking across rooms/surfaces remains additional work. Speech-triggered replay can be delivered earlier if the physical baseline succeeds, but must remain labeled replay.

The prior speech stack has not been recovered. Accessible recent/archived task listings did not identify it. Read-only inspection of the local `woz-demo-stack-main` README and relevant source found a GO2/SO-101 demo, not evidence of the reported Temi/Reachy SLM system. No external project files or running services were changed. Recover deployed service/source/version information on the actual Mac mini/Ubuntu when available; do not assume a new official conversation-app version matches the old installation.

Integration requirements: exclusive head/body motor ownership; suspend conversation gestures during walking; a robot-local expiry/guardian for network loss; deterministic finite requests with deduplication; direct human-stop handling outside SLM reasoning; explicit Mini/Temi routing and rejection of self/TTS-triggered motion. Motor/tilt/timeout limits must be established from the installed hardware and measurements, not copied from GIF/model defaults. The stock model's fixed foot remains a blocker for credible locomotion RL.

## Latest evidence — user GIF

User supplied `/Users/kadir/Downloads/1777717739273.gif`, describing Quest 3 controller motion that made Reachy inch forward like a penguin. Preserved an unmodified copy under `media/user-evidence/2026-10-28-quest-wobble/original.gif`; SHA256 `188785a740e8613d9d3829ca45b2702cfc95783f56e609038703073ffbe51b14` matches the source. 800×800, 63 frames, 80 ms/frame, 5.04 s GIF loop. Playback timing is not verified source-camera timing.

All frames were visually inspected. Head turning/tilting accompanies controller orientation changes; the whole body rocks. The base appears to advance toward the laptop in the later segment. One visible topple develops at frames 18–22 (GIF 1.44–1.76 s), the operator reaches for the fallen robot at 28–29, and the transition 29→30 is a reset/discontinuity. Do not claim autonomous recovery, infer a fall rate, or measure displacement across that boundary or the GIF loop. A cable is visible; function and tension remain unknown. Camera movement, no spatial calibration and missing commands/states preclude a reliable distance/speed claim.

Evidence status is now **user-supplied physical video, visually reviewed**, beyond the prior verbal report; it is not an independently reproduced or instrumented experiment. Notes: `docs/GIF_OBSERVATIONS_2026-10-28.md`; machine-readable annotations/metadata and full-frame contact sheets are beside the preserved original. Existing synthetic fixture results remain synthetic and separate.

New reproducible command:

```sh
.venv/bin/python experiments/inspect_user_gif.py /Users/kadir/Downloads/1777717739273.gif media/user-evidence/2026-10-28-quest-wobble
```

Pillow 11.3.0 was installed only in project `.venv`; `experiments/requirements-media.txt` records it. The extractor was run successfully and checked the original's hash. No learned tracking or fabricated base-pose measurements were added. Next capture should preserve coupled turning/tilting and document the transition into tipping, with external foot tracking and the original cable condition recorded.

## User clarifications preserved

- Robot is **Reachy Mini Wireless**, used on a **flat surface**.
- User made it travel by fast head swinging, wobbling left/right. This is an observed behavior reported by the user; it is not dismissed because the stock product is described as stationary.
- User supplied **https://github.com/kadiru/Emcee** as the likely Quest teleop source.
- User does not remember where the robot-facing bridge ran; it may have been custom code or existing Reachy software. Do not repeatedly ask the same location question. Work with the original machine/robot when available.
- A tabletop and cable are visible in the supplied GIF. Exact surface material, attachments, cable function/tension, installed SDK version and quantitative motion results are still unknown. “Can walk anywhere” has not yet been evaluated as directional control across surfaces.

## Concrete evidence and outputs

**Implemented locally:** JSONL recording contract/validator; Emcee input parser and callback capture hook; read-only HTTP state logger; per-cycle external base displacement, forward/lateral motion, yaw and repeatability analysis; gap rejection; falls/abort/assistance accounting; matched-condition independent-trial comparison; offline target replay schedule generation. Code: `src/reachy_locomotion/`. Recorder/analyzer uses standard-library Python; optional MuJoCo dependencies are isolated in `.venv`.

**Recovered from Emcee source:** pinned commit `76ab2fdef8e872d36c0bfd0c866570e512a2aaad`. `unityWin/EmceeXR/Assets/AgoraTemiViewer.cs` sends right-controller orientation, trigger/A/B/grip and thumbsticks via Agora, default 50 Hz. `pythonReachy/reachy_teleop_rtc.py` only decodes/prints. No actual SDK target mapping was found in the current snapshot. Remote branch check found only main; older history and deployed/uncommitted files were not searched. No Agora connection was made. The sender keeps button states on tracking loss; recovering a receiver must include an independent stale/tracking-invalid stop condition.

**Official model inspected, not a locomotion simulation result:** SDK commit `a9a2a42adcc1abc7d9c90d390889bbebf930c96c`; MuJoCo 3.3.7 successfully compiled the unchanged empty scene. `nq=37`, `nv=30`, `nu=9`, no free joints, zero root-foot collision geoms, root subtree mass 1.17652824 kg, default floor friction `[1,0.005,0.0001]`, all nine selected position actuators force-limited to `[-20,20]` via `perfect_actuator`. The stock model cannot reproduce free-foot travel; base contact, realistic actuators and frame transforms need work before RL. Result: `experiments/reports/stock-mujoco-audit.json`. No rollout/training was run.

**Motor/balance signals researched:** inspected standard state routes expose measured and target joints/poses; Wireless IMU provides acceleration, angular rate, quaternion and IMU temperature. Motor current/load/temperature and contact forces are not in the inspected full-state API. Actual installed-daemon availability and IMU-to-foot calibration remain unverified. Head FK and body-yaw angle are not base displacement.

**Synthetic/mocked only:** `data/synthetic/known-answer.jsonl` has 131 records and three deliberately invented 10 mm cycles. It tests math and formatting, not robot performance. HTTP capture tests use mocked responses. `experiments/reports/synthetic-replay-plan.json` is a schedule, not executed replay, and has `hardware_ready: false`. No external video/tag tracker, synchronized capture integration, real daemon session or hardware replay has been validated.

**Original scaffold tests:** 14 unit tests passed; evidence in `experiments/reports/tests.txt`. They are included in the current 46-test suite above. Includes zero-net wobble, direction-frame transform, tracking gaps, no extrapolation, head/base separation, falls, incomplete cycles, timestamp/nonfinite rejection, no overwritten recordings, read failures and replay provenance. Fixture, source-ledger and model-report consistency checks passed.

**Public sources:** `media/source-ledger.json` has 12 entries: three verified written cannot/can't-walk statements, a qualified HN comparison, two original LinkedIn comments supporting prior motion attempts, four YouTube leads, one unverified X context link and one stale search-snippet lead. **Zero verified spoken clips or video timestamps.** Videos could not be fetched by the web tool; no videos were downloaded/reposted and no creators were contacted. `media/SHOT_LIST.md` has the reveal sequence, with explicit teleoperation/replay/learned labels.

## Runnable commands

Working directory: `/Users/kadir/linkedin/projects/reachy-mini-locomotion`.

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
PYTHONPATH=src .venv/bin/python -m reachy_locomotion validate data/synthetic/known-answer.jsonl
PYTHONPATH=src .venv/bin/python -m reachy_locomotion analyze data/synthetic/known-answer.jsonl --output experiments/reports/synthetic-analysis.json
PYTHONPATH=src .venv/bin/python -m reachy_locomotion replay-plan data/synthetic/known-answer.jsonl --output experiments/reports/synthetic-replay-plan.json
PYTHONPATH=src .venv/bin/python -m reachy_locomotion audit-sim --output experiments/reports/stock-mujoco-audit.json
```

After filling out `data/session-template.json` and identifying the actual daemon address (not run on hardware here):

```sh
PYTHONPATH=src .venv/bin/python -m reachy_locomotion capture-state --url http://REPLACE_WITH_ROBOT_IP:8000 --metadata data/session-template.json --output data/physical/first-state-check.jsonl --seconds 10 --hz 20
```

This is read-only; it does not wake the robot or enable motors. State polling alone neither captures every submitted command nor measures world displacement. Coordinate command/state/video capture through one Recorder and measured clock mappings before claiming synchronization.

## Immediate next action

Recover the robot-facing bridge from the original Wireless robot or demo computer; preserve its code and installed versions. If unavailable, label a reconstructed mapping explicitly and verify axes/neutral/limits at low amplitude. Then record one short rest → manual wobble → rest trial with simultaneous submitted commands, measured joints/IMU, fixed external foot tracking and side-view video. Quantify net base travel against static drift/uncertainty before proceeding to supervised replay.

The reconstructed first-session implementation is now ready for that commissioning workflow; use `docs/FIRST_RECORDING_SESSION.md`. Hardware replay and trained policies are still later milestones.

No permission or auto-review rejection remains outstanding. Missing hardware access and the deployed mapping limit the physical next step; the local tools are ready for instrumentation work.

## Next milestones and references

1. One measured teleop reproduction; identify rocking vs sliding and any cable/surface contribution.
2. Supervised replay; proposed 5 independent trials × 10 cycles, include all failures and compare matched conditions.
3. Compact imitation/parameter optimization after calibrated simulator replay.
4. RL only after free-base/contact/actuator model validation; no shared-GPU work until coordinated with the existing humanoid job owner.

`README.md` has usage. `docs/RECOVERY.md`, `docs/RECORDING_SCHEMA.md`, `docs/EXPERIMENT_PLAN.md` and `docs/SIM_AUDIT.md` contain sources, capture design, reward/evaluation definitions and limits.

Reference downloads are project-local: Emcee (~1.2 GB, vendor assets included) and Reachy SDK (~40 MB after 65 official LFS assets were hash/size verified). Initial MuJoCo compilation failed on unhydrated LFS meshes; helper `experiments/fetch_mujoco_assets.py` resolved this. `experiments/requirements-audit.lock` records the isolated environment. Sibling projects and existing humanoid training were untouched.
