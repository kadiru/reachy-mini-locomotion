# Reachy Mini — Quest to Locomotion — Status

## November 19 — faster penguin gait and repaired known failures

Trained on the previous failing models and shortened state-gated pauses. All
**26 previously seen walking cases pass** at 0.25 ms; the four formerly failing
cases also pass at **0.125 ms**. A new frozen split (seed 2026111992) gives
**14/16 passes**: unseen-3 left hits the 35° guard; unseen-7 left stays upright
but exhausts turn attempts, advances only 6.33 mm and ends 23.14° off heading.
No tuning followed that split. Do not claim universal no-fall robustness.

Selected policy: `configs/penguin-faster-feedback-trained.json`. The primary
catch uses 14° predicted signed roll, 80 ms prediction and a 20 ms response
delay. Physical forward-head return starts 0.6 s after catch over 0.4 s; yaw
return is 0.4 s. Shorter phase floors retain the same low roll/rate checks.
A regular fallback is selected after <3° observed signed turn. A learned rescue
launch is used only if that fallback also gives <5°, with older roll samples.
Selection uses feedback and known sensor age, never hidden model parameters.
The initial age-only selection policy is retained as experimental evidence and
is superseded. Gain-transition, prediction-age and head-settling variants remain
optional, disabled in the selected config. Earlier policies are preserved.

**Best nominal video:** left start, four complete pairs / eleven bursts,
**112.05 mm in 30.44 s of stepping**, **3.681 mm/s** versus 2.925 (**+25.8%**).
Minimum bilateral roll 15.26°, peak rim-height difference 29.77 mm, maximum
pair-end heading 7.52°. Burst intervals are typically 2.76 s versus 3.78 s.
The full 36 s simulation remains settled; video shows motion plus one second
of rest at 1×. The right start is also valid: 80.96 mm in 22.20 s, 3.647 mm/s,
0.89° maximum pair-end heading, four pairs / eight bursts.

Both four-pair directions pass at 0.125 ms. The selected left run gives
111.90 mm, 3.676 mm/s and 7.48° heading, a 0.14% speed change. Some stress-case
progress remains timestep-sensitive (fresh-3 left: 24.90 vs 20.73 mm), although
both checks pass. Waist target remains zero; its measured angle deflects
−2.63…+2.62°, with 0.0787 Nm peak holding torque. Physical motor/profile limits,
mass/contact model and all failure guards are unchanged. No robot, shared GPU,
Nimbus or EY work was touched; everything remains uncalibrated simulation.

Media: `media/simulation/faster-penguin-best-2026-11-19.mp4` and
`media/simulation/faster-penguin-2026-11-19-chart.png`.
Method and commands: `docs/FASTER_PENGUIN_2026-11-19.md`.
Frozen qualification: `experiments/reports/motor-training-faster-balance-feedback-qualified-2026-11-19.json`.
Fine failure checks: `experiments/reports/motor-training-faster-balance-finer-failures-2026-11-19.json`.
Artifact receipt: `experiments/reports/motor-training-faster-balance-artifact-receipt-2026-11-19.json`.
**105 tests pass** in `experiments/reports/repository-tests-faster-balance-2026-11-19.txt`.

Next: promote the two newly exposed cases to development; improve catch recovery
and weak-step selection without reducing the speed already achieved. Reserve
another unseen split. Do not execute on hardware before motor/contact/sensing
calibration. Personal Git identity remains kadiru / Kadir Uyanik.

## November 19, 2026 — bilateral feedback and composed steps

Implemented and trained signed roll/rate feedback with bounded head correction,
predictive catch/yaw timing, checked-IK projection, physical head return, and
heading-based sequencing. Two learned launch patterns are selected from observed
progress, never hidden model parameters. Waist target stays zero with bounded
holding torque; motor limits/profile, joint guards and old walking configs are
unchanged. New trials use the 35° guard. No physical state reset between bursts.

- **Recovery: 26/26** at 0.25 ms, both sides of five development models plus
  sixteen fresh noisy/delayed-sensor checks. Frozen config:
  `configs/balance-recovery-trained.json` (fresh seed 2026111963).
- **Pivot + walking development: 20/20** at both 0.5 and 0.25 ms. Five models ×
  both initial sides × isolated pivot/full-pair walking. Frozen policy:
  `configs/penguin-feedback-experimental.json`.
- **Broader frozen qualification: 13/16 pivots and 13/16 walking trials pass**
  at 0.25 ms (seed 2026111971). Fresh-3 left and fresh-7 left hit the 35° guard;
  fresh-7 right walking also hits it on the later left half. Fresh-5 right pivot
  recovers but has only 4.73° net edge-supported turn, below the 5° gate. These
  are failures, not no-fall successes. No complete stress robustness is claimed.
- **Finer nominal checks: 4/4** at 0.125 ms, both starting sides and both modes.
  Separate two-pair test moves 89.16 mm with 1.03° maximum pair-end heading.
- **Video:** left and right recovery, then two complete pairs / eight bursts.
  The 0.25 ms gait advances **88.64 mm**, 44.32 mm per pair, with 15.27° minimum
  bilateral roll, 29.78 mm peak rim-height difference, and pair-end headings
  −0.32° / −0.60°. Eight bursts finish at 30.30 s; recording continues to 32 s.
  Speed is 2.93 mm/s through the final step, 2.77 mm/s including final rest.
  This deliberate settling gait is slower than the preserved rapid mode.

The video is `media/simulation/bilateral-penguin-2026-11-19.mp4`, 42.12 s at
25 fps / 1×, including both recovery demonstrations. It renders saved physical
states. Companion `-chart.png` displays every qualification outcome. In the gait,
waist torque peaks at 0.0780 Nm and its measured angle deflects −2.48…+2.44°;
a fixed target is not a welded or unpowered waist. Peak body tilt is 18.46°.

A further emergency neutral-return controller was screened after promoting the
two new failing conditions to development. It does not fix the complete sequence
and introduces another joint-guard/missed-goal failure. Kept as an optional
experimental feature, **not selected in the frozen/video policy**. Early aborted
screen evidence is retained as incomplete; no failure is discarded or relabeled.
All motor/contact/sensing assumptions remain uncalibrated, and hardware-ready
flags remain false. **100 tests pass** in
`experiments/reports/repository-tests-bilateral-final-2026-11-19.txt`.

Evidence and commands: `docs/BILATERAL_BALANCE_2026-11-19.md`;
`experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json`;
`experiments/reports/motor-training-balance-neutral-qualified-2026-11-19.json`;
`experiments/reports/motor-training-bilateral-artifact-receipt-2026-11-19.json`.
No hardware, shared Ubuntu GPU, Nimbus or EY work was touched. GitHub CLI access
resumed; authenticated login verified as `kadiru`, and the remote history was
fetched without changing local work. Personal author/committer guards remain on.

Next: retrain the phase transition and recovery from partially completed pivots
using the newly exposed left-side failures; qualify on a new frozen unseen split.
Then reduce the settling pause while retaining the strict gates. Do not deploy
this policy to hardware or claim it passes every stress test.

## November 19, 2026 — sideways recovery and edge pivot primitives

Latest instruction supersedes speed optimization: learn to lift/tip sideways,
recover, and pivot on an edge before composing bigger penguin steps. User
permits the waist to hold a fixed angle. Implemented an isolated diagnostic
simulator with **constant zero waist target**, bounded holding torque, 20 ms
delayed roll-triggered head response, separate floor-foot/torso/waist rotation,
and contact-material-point slip. The existing walking evaluator, 35° walking
guard, motor model, motor profile and previous gait configs are unchanged.

Selected recordings at **0.25 ms**, each requesting eight seconds:

- **Fall:** continued head pumping reaches 69.75° roll and first nonfoot contact
  at 6.505 s. This is an intentional simulation-only diagnostic.
- **Recovery:** same launch and identical qpos/qvel through its 1.48 s trigger;
  changed head response reaches **30.03° roll**, **55.59 mm opposite-rim height
  difference**, then settles upright with **2.82 mm center drift**. This prevents
  the continuing launch's eventual fall; it does not establish a point of no
  return or prove the robot was already committed to falling at the trigger.
- **Edge pivot:** different recovered launch plus 40° head-yaw stroke/hold/return;
  **−17.78° final base yaw**, **−14.81° during 1.02 s edge support**, 22.23° peak
  roll, **16.79 mm drift / 4.37 mm estimated slip**. Sway-only comparison gives
  −10.58° edge turn; slower yaw gives −12.55°. Holding waist torque peaks at
  0.0533 Nm; measured waist angle remains within −2.23…+1.31°.

Nominal fall/recovery/pivot outcomes persist at 0.5, 0.25 and **0.125 ms**.
Fine catch search: 324 trials, 127 recovered and 197 fell; 11/12 selected
recovered candidates pass finer validation. For the selected launch all tested
late triggers at 22–38° fall, while many earlier triggers recover. The timing
and angular velocity differ, so **there is no universal measured tipping angle**.
Twelve pivot candidates pass the explicit edge-turn/drift/slip gate at both
0.5 and 0.25 ms. Coarse timestep artifacts and all guarded failures are retained.

**Unresolved robustness:** fresh four-model probe gives catch recovery 3/4 and
pivot recovery 3/4, but **0/4 pivot-gate passes**; one variant falls for both
policies and the other pivots turn too little. Mirroring the catch falls.
Mirroring the pivot recovers at 0.25 ms with +29.00° final yaw, not yet qualified
at multiple timesteps. No robust bilateral walking, maximum possible lift or
physical transfer is claimed. No hardware/shared GPU/Nimbus/EY work was touched.

Frozen configs: `configs/primitive-{fall,catch,pivot}-experimental.json`.
Final report: `experiments/reports/motor-training-primitives-final-2026-11-19.json`.
Video: `media/simulation/primitive-curriculum-2026-11-19.mp4`, real-time three-part
comparison. Chart: same prefix with `-chart.png`. Rendering uses saved states,
including the actual terminal contact pose; no simulated bounce is fabricated.
An independent CLI replay matches every saved catch field/trace/state exactly.
**89 tests pass**; log `experiments/reports/repository-tests-primitives-2026-11-19.txt`.
Artifact receipt records media hashes, 566 frames at 25 fps (22.64 s), software
versions, the exact replay check and 17 unchanged walking/motor files.

Publication pending: GitHub connector verified `kadiru` and remote `main` at
`ca2d46eb20eb41cca8f8a9eef9ad4aaa63bbc252`, matching the local parent. Mac HTTPS
fetch/API and SSH connection attempts timed out. Changes are committed locally
with the verified personal author/committer; no push or new GitHub CI run has
completed. Resume by verifying local GitHub authentication and fetching before
pushing the one local commit. Do not bypass the identity guard.

Full method, phase counts, boundary map, comparisons and limitations:
`docs/PRIMITIVE_CURRICULUM_2026-11-19.md`. Reproduce from this project directory:

```sh
.venv/bin/python experiments/run_primitive.py configs/primitive-catch-experimental.json --record --output experiments/generated/my-catch.json
.venv/bin/python experiments/render_sway.py experiments/generated/my-catch.json --output experiments/generated/my-catch.mp4 --slowdown 1 --title 'CATCH AND RECOVER' --diagnostics --frame-path --follow-front
```

Next: add signed roll and roll-rate feedback, train both directions across
initial phases/model variants, qualify recovery in angle/rate space, then
alternate the edge pivots to advance the unloaded side. Restore the user's
20° **full-cycle** heading criterion when composing a walking gait.

## November 19, 2026 — larger rocking demo recorded; continuous qualification fails

User confirmed that 20° applies after every complete left/right cycle, relative
to startup; larger within-step torso turns are allowed. Added observational
cycle metrics and delayed same-phase heading feedback. Physical motor limits,
software profile, joint/IK guards and the previous rapid gait remain unchanged.

Trained on four local CPU workers: **1,719 exploration/refinement rollouts**,
followed by 110 qualification rollouts. Neither the original nor explicitly
provisional long-run selection passed. Preserved those failures and froze the
best qualified **twenty-second demonstration**, rather than calling it a
continuous-walking solution. Frozen configs:
`configs/{sway,heading}-rocking-experimental.json`.

Recorded nominal trial: **1.004 m in 20.16 s, 49.81 mm/s, 15.94 mm/full cycle**,
with 6.14° median bilateral roll, **8.56° peak body tilt**, both edges loaded
within every cycle, and **11.57° maximum cycle-end heading**. No falls or
airborne intervals. The steps are 26.2% longer than the rapid mode's 12.63 mm,
and peak tilt is 3.52× larger; the rapid mode remains faster at 57.39 mm/s.
The existing nominal 5 V hypothesis and 0.416 Nm head caps are unchanged.

At 0.5/0.25 ms, speed is 45.78/45.01 mm/s, stride 14.65/14.40 mm/cycle, and
cycle-end heading stays within 15.18°/15.66°, upright throughout twenty seconds.
These miss the original 15 mm style goal; instantaneous yaw up to 23.67° is
allowed under the new rule. In both 40- and 120-second requests the selected
controller drifts past 20° and stops at an invalid IK target after **38.04 s**.
The invalid command is not executed. Twenty-two smaller-trim/phase-steering
variants complete forty seconds at both timesteps, but none stays on course.

After freezing: **8/16 fresh model trials complete, 0/16 pass heading**. The
other eight stop at the IK guard. No fall/nonfoot-contact termination occurs,
but incomplete guarded runs are failures, not successful no-fall trials.
This gait is **not robust or hardware-ready**. No physical robot, shared GPU,
Nimbus or EY project was touched.

Video: `media/simulation/rocking-steps-2026-11-19.mp4`, real time, 529 frames at
25 fps, 1280×580; startup included. Early/late/final frames were visually checked.
The ordinary CLI replay exactly matches every recorded result field and state.
**85 tests pass**; log `experiments/reports/repository-tests-rocking-2026-11-19.txt`.
Details and runnable commands: `docs/LARGE_ROCKING_2026-11-19.md`. Compact evidence
and hashes: `experiments/reports/motor-training-rocking-{summary,artifacts}-2026-11-19.json`.

Next: preserve the two distinct modes and this short demonstration, then identify
a steering response that stays inside the reachable head workspace over long
runs. Train against long-horizon drift and model variation from the outset;
the current fresh suite is now evaluation evidence and must not be reused as an
untouched holdout. Physical motor/contact/heading calibration is still required
before considering robot execution. Mode transitions remain unimplemented.

Reproduce the frozen numerical trial from this repository root:

```sh
.venv/bin/python -m reachy_locomotion.sway \
  --config configs/motors-speed-trained.json \
  --gait configs/sway-rocking-experimental.json \
  --feedback configs/heading-rocking-experimental.json \
  --sensor configs/heading-sensor-assumed.json \
  --warmup-cycles 3 --cycles 63 \
  --output experiments/generated/rocking-replay-next.json
```


## November 19, 2026 — two walking styles defined; large edge steps remain a separate target

User clarified the intended endpoint: preserve rapid, small steps and also
develop a larger sway/turn/stop gait with deliberate torso roll/yaw and alternating
base-edge support. Recorded separate objectives in `docs/GAIT_STYLES.md` and
BRIEF.md. No new gait has been trained in this clarification turn; controller,
motor limits and fall checks are unchanged.

Rechecked the existing 20-second fast trajectory: 4.545 Hz, **12.63 mm per complete
left/right cycle**, 2.43° maximum body tilt and a 4.33 mm maximum side-rim height
proxy. Its 50 Hz samples show 458 left-edge and 542 right-edge support frames,
180 direct left/right support changes, and about 5.93° mean within-cycle yaw
excursion. This supports classifying it as the rapid small-step baseline, rather
than claiming the large-tilt gait has already been reproduced. Evidence:
`experiments/reports/motor-training-gait-styles-baseline-2026-11-19.json`.

Attempted to replay Vimeo; both the in-app browser and Chrome displayed
"Unable to play media". Used the earlier Vimeo review and re-inspected the saved
GIF contact sheets (frames 0–20 and 42–62). The latter visibly show coupled head
turning/tilting and torso rocking; hidden contacts and camera motion still prevent
metric physical tilt/step/force estimates. No new Vimeo playback is claimed.

Heading clarification is pending: the user has been asked whether the 20° bound
can apply after a full left/right cycle while allowing larger intermediate torso
swings. Until answered, preserve the existing instantaneous bound. Next: add
separate cycle-progress, support-transfer and within-cycle-versus-net-heading
metrics to the large-step training objective, then search its gait family under
the unchanged actuator/joint/no-fall constraints. Keep both mode winners and
evaluate transitions separately; no mode-switching controller exists yet.


## November 19, 2026 — larger-yaw headshake gains another 19%, two-minute check passes

The user requested more forward speed from larger swings/sharper braking while
keeping realistic motor limits, no falls and up to 20° turning. **Both physical
and software motor limits remain unchanged** from the 48.19 mm/s result.
Recorded 693 candidate evaluations on four local CPU workers, including wider,
slower swings and explicit short-braking strokes. The first refinement stopped
after seven complete generations on a reproduced floating-point angle-boundary
bug; those results were preserved and explicitly recovered. A 1e-12° arithmetic
tolerance fixes that bug without changing motor/IK limits or valid trajectories.

Selected policy `0-same`: **1.149 m in 20.02 s, 57.39 mm/s**, **19.09% faster**
than the prior gait. Heading stays within 14.39°, with 23.5 mm lateral travel.
The 40.04 s run moves 2.280 m, maximum heading 15.18°. An additional **120.12 s**
check moves **6.754 m** at 56.23 mm/s, still within 15.18°, without falling.
Maximum foot tilt is 2.43°; minimum motor joint-limit margin is 0.527 rad.
At 0.5/0.25 ms, speeds are 57.23/57.08 mm/s (within 0.6% of 1 ms).

Measured yaw span increases from 50.62° to **55.88°** (+10.4%), while measured
roll span decreases. Retiming roll/yaw reversals matters more than added waveform
sharpness in matched ablations. Explicit trapezoidal brake/hold strokes did not
win. The selected gait keeps the existing 10° heading deadband and 4.545 Hz actual
cycle rate. It remains edge-supported rocking: **no airborne interval detected**.

Selection used 55 twenty-second rollouts, eight fine-timestep checks and four
40-second checks before freezing. All **16 fresh model trials remain valid and
upright**, with positive forward progress, but only **6/16** stay within 20°.
Worst heading is 114.43°; speeds range 2.56–95.58 mm/s. This set uses a new seed,
so it is not a paired robustness comparison with the preceding experiment.
No hardware transfer or motor-response identification has been performed.

Evidence: `docs/FASTER_HEADSHAKE_2026-11-19.md`,
`experiments/reports/motor-training-faster-{selection,summary}-2026-11-19.json`,
the complete and interrupted search receipts, and
`media/simulation/faster-headshake-2026-11-19.mp4` (1× playback, following head
close-up plus fixed side view). Recording exactly matches the selection run;
start/late/final video frames were visually checked. The old policy/video remain.
Frozen gait/feedback: `configs/{sway,heading}-faster-trained.json`; motor config
remains `configs/motors-speed-trained.json`. **81 tests pass**, including the
boundary regression. Commands are in the new document.

Next: preserve this faster nominal baseline while improving steering across
model variations; identify the installed motor/profile/contact response before
hardware trials. No robot, shared GPU, Nimbus or EY project was touched.


## November 19, 2026 — speed-first headshake reaches 0.96 m in 20 seconds

Latest user priority: maximum forward speed without falling, allowing up to
20° heading change. Removed smoothness/lateral penalties from the objective.
Compared the prior gait, the original high-frequency headshake and explicit
rapid-stroke/brake/hold waveforms in 612 candidate evaluations on four local CPU
workers. Selected on 96 longer/finer rollouts, then froze the policy before 16
fresh stress runs. No robot or shared GPU was used.

**Selected 1 ms run:** 964.8 mm in 20.02 s, **48.19 mm/s**, **2.73×** the prior
fast gait on the same physical model. Maximum heading from startup is 17.29°.
The 40.04 s run moves 1.93 m without falling; maximum foot tilt is 3.22°.
At 0.5 ms, speed is 48.47 mm/s (0.57% difference) and heading stays within 16.62°.
The 20-second saved replay matches selection exactly; the video plays at 1×.

Kept the nominal 5 V hypothesis, 1.475 kg mass, 0.416 Nm head torque caps,
motor back-EMF/no-load-speed envelope, joint limits and contact model. Changed
software target speed/acceleration/filter from 6 rad/s, 60 rad/s², 5 Hz to
10.7 rad/s, 360 rad/s², 25 Hz. These are model/software choices, not verified
firmware limits. Body yaw remains zero. Actual gait frequency is 4.545 Hz;
bounded heading feedback uses a 10° deadband. The winning waveform uses the
existing smooth swing family; explicit trapezoidal brake/hold candidates were
tested but did not win. Ablations show fast command timing and coupled roll/yaw
are important, without establishing braking alone as the cause.

Mean progress is 10.6 mm/cycle, with single-edge support in 99.6% of sampled
frames. **No true airborne interval was detected** using substep foot-bottom
clearance plus normal force. This is fast rocking locomotion, not a verified hop.
All 16 fresh trials stay upright and move forward, but only **6/16** pass the
20° heading constraint; worst heading is 84.15° and speeds range 1.55–73.98 mm/s.
Hardware response and the heading estimator remain unidentified; no sim-to-real
success is claimed.

Evidence: `docs/SPEED_FIRST_2026-11-19.md`,
`experiments/reports/motor-training-speed-summary-2026-11-19.json`, detailed
search/selection/demo reports, and `media/simulation/speed-first-2026-11-19.mp4`.
Frozen policy: `configs/{sway,motors,heading}-speed-trained.json`.
All **80 tests pass** in `experiments/reports/repository-tests-speed-final-2026-11-19.txt`.
Reproduction commands are in the speed-first document. Next: improve steering
under the identified difficult model variations while retaining this speed
baseline, and identify actual motor/profile/contact response before transfer.


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
