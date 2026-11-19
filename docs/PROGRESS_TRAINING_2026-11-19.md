# Faster compound head motion — November 19, 2026

The user requested faster forward movement with less turning, explicitly making
progress more important than smoothness. They described head roll plus yaw,
brief support on one edge, and head deceleration transferring reaction torque
to the body. [The opening video was reviewed again](VIMEO_REVIEW_2026-11-19.md).

## Best nominal result and its limits

**The selected fast gait advances 36.4 cm in 20.52 s at 1 ms physics**, versus
5.09 cm for the previous smooth gait over exactly the same duration and startup
protocol: **7.15× faster**. The 2 ms validation gives 37.1 cm, a 2.0% displacement
difference. The displayed video is the nominal 1 ms run at real-time speed,
including 1.62 s of startup before the measured interval.

| Matched 20.52 s, 1 ms physics | Previous smooth | New fast |
|---|---:|---:|
| Forward distance | 50.9 mm | 363.9 mm |
| Lateral distance | -8.8 mm | -27.6 mm |
| Forward speed | 2.48 mm/s | 17.74 mm/s |
| Net heading change after startup | -5.43° | -7.76° |
| Maximum heading excursion after startup | 5.43° | 9.03° |
| Maximum heading excursion including startup | 5.58° | 11.52° |
| Peak foot tilt | 0.81° | 3.54° |
| Opposite-rim clearance | 1.19 mm | 5.90 mm |

The new gait gives up some straightness compared with the slow gait to gain
progress. It turns much less than the earlier **fast** learned gait: over matched
12.42 s, that baseline turns 115° and travels 42 mm forward/133 mm sideways,
whereas the selected gait turns 6.1° and travels 219 mm forward/13 mm sideways.
All these comparisons use the same newer motor model.

The fast run's peak head-motor torque is 0.4055 Nm, below the unchanged 0.416 Nm
cap; maximum head-joint speed is 6.02 rad/s, below the nominal 10.79 rad/s no-load
endpoint. No measured step reaches the 99%-of-cap saturation threshold. RMS
head-motor torques span 0.101–0.189 Nm; no thermal/continuous-duty conclusion is
made. Saved actual joint positions retain at least 0.219 rad margin to the
inspected joint limits. Actual head roll spans -31° to +29°, yaw -50° to +54°,
and pitch +15° to +29°. Larger actual yaw excursion is visible in the demo.

**Robustness is not solved:** all 12 fresh fast-policy stress cases advance and
remain numerically valid, but **0/12 pass the full turning/drift criteria**.
Peak heading excursion ranges 12.6–110°. The previous smooth gait passes 10/12
on that same stress set. The fast video is consequently a nominal-model result,
not a robust sim-to-real policy. A separate conservative search produces about
6.1–6.6 mm/s nominal speed; its receipt uses a different heldout set, so its
pass count must not be compared directly with this fast-policy set.

### What the mechanism checks show

At 50 Hz, single-edge support occurs in 76.1% of measured samples. All sampled
yaw-slowing events occur during single-edge support. This supports the proposed
timing in this simulation, without proving the real robot's force mechanism.
Removing the yaw stroke reduces speed to about 4.05 mm/s; shifting yaw phase by
90° gives 3.97 mm/s. Removing roll plus lateral translation leaves about
0.001 mm/s. Removing waveform dwell shaping retains 17.34 mm/s, close to the
selected 17.7 mm/s. Thus coupled motion and phase matter substantially; an
aggressive dwell is not necessary for this model's selected solution.

The learned stroke uses 38.6° roll amplitude, 26.0° yaw command amplitude,
93.3° relative phase, modest yaw shaping (0.542) and approximately 1.852 Hz.
The larger actual yaw arises from the simulated response, not a target of 54°.
That amplification/phase lag is another reason motor response needs physical
identification before replaying this on hardware.

Artifacts:

- [Real-time video](../media/simulation/progress-fast-2026-11-19.mp4)
- [Compact summary](../experiments/reports/motor-training-progress-summary-2026-11-19.json)
- [Selected fast gait](../configs/sway-progress-fast.json)
- [Conservative alternative](../configs/sway-progress-conservative.json)
- [Complete search](../experiments/reports/motor-training-progress-2026-11-19.json)
- [Fast selection, validation and stress outcomes](../experiments/reports/motor-training-progress-fast-2026-11-19.json)
- [Matched demonstration metrics](../experiments/reports/motor-training-progress-demo-2026-11-19.json)
- [Contact/braking phase analysis](../experiments/reports/motor-training-stroke-2026-11-19.json)

624 candidate evaluations produced 1,200 training rollouts. All 78 invalid
training rollouts are retained (71 joint-limit and seven nonfinite IK-result
rejections). Development, selection, heldout and ablation checks add 264 rollouts,
plus three recorded demonstrations. **67 local tests pass** including periodic
bounded stroke/startup, combined bias/amplitude limits, spin-rejection criteria,
and non-invasive recording of contact and angular momentum.

## Controller and unchanged motor bounds

`Gait` now supports independently shaped roll/yaw strokes and small roll/yaw
biases. The waveform is `tanh(k*sin(phase))/tanh(k)`; `k=0` retains the original
sine. Increasing `k` gives a shorter swing and longer dwell near its endpoints.
Yaw phase places its deceleration relative to the roll/lateral stroke. Roll
amplitude plus bias is bounded by 40°, yaw by 60°; official IK and motor joint
limits reject unreachable combinations.

The experiment does **not** increase motor torque, reduce mass or artificially
move the base. It retains `configs/motors-datasheet.json`: nominal 5 V, per-motor
DC/PWM approximation, 0.416 Nm head torque cap (80% of the published 5 V stall
value), torque-speed response, 50 Hz commands, latency, and finite command speed
and acceleration. Published specs are distinguished from unmeasured dynamics in
[MOTOR_LIMITS.md](MOTOR_LIMITS.md). Actual battery-bus voltage is still unknown.
A sharp target cannot make the physical model stop instantaneously.

## Training and selection

`train_progress.py` performs CPU CEM parameter search (not RL): 12 generations
of 36 candidates in the nominal model, then 8 generations of 24 candidates over
four development conditions. The first phase produces the fast candidate pool;
the second seeks a more conservative alternative. Development conditions are
nominal 5 V at 1/2 ms, 5.5 V with friction 0.6, and 6 V with friction 0.8 and
40 ms command latency. Total mass stays 1.475 kg.

The objective rewards forward speed with no acceleration/smoothness penalty.
Feasible trajectories rank above infeasible ones. Constraints are <=10° maximum
heading excursion throughout the measured interval, lateral excursion <=30% of
forward displacement plus 3 mm, peak foot tilt <=18°, numerical/contact validity,
and >=0.5 mm/s progress. Measuring maximum unwrapped heading prevents a full spin
that returns to its starting orientation from passing as straight travel.

The conservative phase can sacrifice speed to cover uncertain parameter sets.
The user prioritized progress, so `select_progress.py` separately selects from
the **nominal exploration archive**, with no use of held-out results: top 24
nominal candidates are evaluated for at least 12 s at both 1 and 2 ms. The best
six eligible candidates then receive at least 20 s checks at both timesteps.
The fastest minimum speed across timesteps is preferred among passing candidates.
If none passes the 20 s check, the script explicitly records that fact and keeps
the best 12 s candidate. The chosen policy is frozen before fresh stress tests.

Whole-cycle measurement windows are rounded up from the requested duration;
reported speeds use actual duration. Three startup cycles are excluded, shown
in the saved video, and not counted as progress. Nominal and stress outcomes are
reported separately: the fastest nominal candidate is not called a universally
robust policy. Six fresh stress configurations (seed 591191), each at 1 and 2 ms,
vary voltage, gain, friction, mass placement, delay and reflected rotor inertia.
Baseline gaits run under the same conditions and measurement protocol.

## Mechanism checks

Saved states include phase, joint velocities, motor torque/velocity, left/right
foot normal forces and head-subtree angular momentum. MuJoCo's
[`mj_subtreeVel`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-subtreevel)
computes the momentum; its reference is the subtree's own COM, in global axes,
as specified for [subtree angular momentum](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-subtreeangmom).
It must not be subtracted directly from whole-robot momentum about a different
COM. Ground contact supplies external forces and moments.

`analyze_stroke.py` measures concurrent yaw deceleration and single-edge support,
plus signed net head-motor power. The video's “SLOWING” label describes a reduction
in yaw speed at 50 Hz; it is not an experimentally identified torque impulse.
Ablations remove yaw, remove the lateral roll/translation stroke, remove dwell
shaping, or shift yaw phase by 90°. They test the simulated coupling after policy
freeze and never select a new policy. No hardware causal claim is made.

## Reproduction

Install the project sim extras and pinned upstream assets first. Use new output
paths; no existing experiment is overwritten. Runs use at most four local CPU
workers; no shared GPU or robot is contacted.

```sh
.venv/bin/python experiments/train_progress.py --output experiments/reports/motor-training-progress-new.json
.venv/bin/python experiments/select_progress.py experiments/reports/motor-training-progress-new.json --output experiments/reports/motor-training-progress-fast-new.json
.venv/bin/python experiments/render_sway.py experiments/generated/motor-training-progress-fast-new-states.json --output media/simulation/progress-new.mp4 --slowdown 1 --title 'COMPOUND HEAD SWING' --diagnostics
.venv/bin/python experiments/analyze_stroke.py experiments/generated/motor-training-progress-fast-new-states.json --output experiments/reports/motor-training-stroke-new.json
```

The video replays actual saved physics states with fixed cameras, a 5 cm grid,
forward displacement, heading change, support side and yaw deceleration labels.
It does not rerun physics, retime the movement, or animate the base by hand.

The saved 20.52 s demonstration can be regenerated with the selected gait,
`cycles=38`, `warmup_cycles=3`, and `configs/motors-datasheet.json` with only
`timestep_s` changed to 0.001. Renderer options for that video are
`--slowdown 1 --title 'COMPOUND HEAD SWING' --diagnostics --frame-path`.
The cameras stay fixed while framing the entire recorded path.
