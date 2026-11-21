# Turning Reachy Mini to a requested heading

The command is a signed change in torso heading from startup. Positive turns left.
Reachy starts upright, rocks onto a rim with its head, turns, and can replant
between strokes. The waist holds the same neutral target throughout. All
results are from the free-base MuJoCo simulation.

## Nominal turns

| Torso request | Settled torso | Net signed base turn on rims | Peak rim lift | Maximum center shift | Strokes |
| --- | ---: | ---: | ---: | ---: | ---: |
| 45° left | 41.4° left | 23.8° left | 31.8 mm | 27.4 mm | 4 |
| 45° right | 44.8° right | 27.5° right | 31.4 mm | 26.6 mm | 5 |
| 90° left | 89.1° left | 51.6° left | 31.8 mm | 47.5 mm | 9 |
| 90° right | 88.4° right | 52.5° right | 31.4 mm | 51.0 mm | 9 |

Both quarter-turn trials took 40.4 seconds including final settling. The
left and right turns accumulated 36.4 and 32.2 mm of material contact slip,
respectively. The measured waist deflected by up to 2.24° across those trials
while its command stayed at zero. [Watch both quarter-turns](../media/simulation/edge-quarter-turns.mp4)
or [view the shorter 45° turns](../media/simulation/target-edge-pivot.mp4).

## How the policy learns and steers

The training run tested 88 head motions on five development conditions, for
440 single-stroke episodes. It varied the direction, amplitude and timing of
head yaw, head offset and pitch, roll frequency, and the roll threshold for
catching the tilt. It retained 59 motions that settled on every development
condition.
The first multi-stroke tests exposed a joint-travel stop when one motion followed
another. Refinement excluded that motion and measured the remaining 58 motions
on two lower-friction conditions, adding 116 episodes. Each motion has measured changes in heading, signed turn
on the rim, center displacement, and peak sensed roll for each condition.

At 50 Hz, the inner controller estimates roll rate from delayed, noisy roll.
It uses those signals to catch the tilt, turn the head and return it to neutral.
Every 4.4 seconds the outer controller reads relative base heading after
landing. It compares the observed turn and roll with the five learned
responses, then predicts which motion will most closely eliminate the remaining
heading error. The cost also penalizes center travel and insufficient rotation
on a loaded rim. The two-frame sensing case has a weight fitted on lower-friction trials. Previously observed turns update the estimate for repeated
motions. A short feedback search on difficult multi-stroke left and right turns selects
the rim-rotation penalty under delayed sensing and an extra inward joint margin after a landing that
changed heading by more than 12.5°. Four parameter pairs ran on eight
development episodes each. The selected delayed-sensing weight was 0.2,
and the extra-margin setting raised the command reserve to 0.10 rad. The policy accepts a measured error within 4.1° or ends after nine
strokes. State evolves continuously through the sequence. This is empirical
policy search with heading feedback.

Ground forces, center position and torso yaw are available to the evaluator.
The policy uses the known sampling delay and simulated observations of roll
and relative heading. A physical estimator for this input still needs
validation. The simulation uses the project's battery motor model, its torque,
speed and acceleration bounds, and a 0.065 rad inward margin for head joint
commands. Supply voltage, mass distribution and friction on hardware are
uncalibrated.

## How a turn passes

- The physical rollout completes and settles. During its final 0.6 s, tilt
  stays below 3°, angular speed below 0.2 rad/s and center speed below 0.02 m/s.
  Both base and torso must stay within 5° of the requested heading in that window.
- Net signed base rotation measured on the loaded rims must account for at
  least 40% of the requested angle. A qualifying sample has at least 10 mm rim
  height difference, more than 0.5 N on the loaded side and less than 0.1 N on
  the other. Both samples bracketing a yaw increment must qualify on the same
  side. Reverse rotation on a rim subtracts from the total.
- Base-center excursion stays within one modeled foot radius (56 mm) throughout.
  Mean integrated slip of the material contact point during rim support stays
  within 15 mm per stroke. These thresholds allow measurable translation.
- The simulator retains the physical joint, closure and floor-contact checks.
  There is no tilt-angle cutoff. A joint-limit termination is recorded separately
  from a fall.

## Qualification and failure conditions

Of 67 episodes, 61 settled within 5° on both base and torso, and 47 passed
every pivot criterion. This includes one zero-heading hold that used no
strokes. Fourteen episodes reached the heading on lower-friction floors
with too little net turn on a single loaded rim. Six right-turn episodes
stopped at the simulated joint-travel guard with uniform mass placement and
40 ms of command delay. No episode recorded a body-floor collision. The
outcome after an early guard termination is unknown.

| Test conditions | Full passes | Settled at requested heading |
| --- | ---: | ---: |
| Nominal, including both quarter-turns and the zero command | 11/11 | 11/11 |
| Two finer nominal physics timesteps | 8/8 | 8/8 |
| Higher-grip floors, including fresh parameters | 16/16 | 16/16 |
| Lower-grip floors, including fresh parameters | 2/16 | 16/16 |
| Uniform mass allocation and two-frame command delay | 10/16 | 10/16 |

The totals mix development retests and independent trials. On the 12 reserved
trials with new combinations and targets of ±24° and ±42°, six passed every
criterion and ten settled at the requested heading. The four slick-floor
episodes turned mostly outside qualifying edge support. The two right-turn
episodes with uniform mass placement stopped at the joint guard.
[Watch all 67 tests with their conditions and outcomes](../media/simulation/target-pivot-all.mp4).

## Reproduce

Run from the repository root with the project environment and pinned Reachy
Mini assets installed. The replay command takes any requested change of
heading between −180° and +180°. The tested range reached 90° in either direction.

```sh
.venv/bin/python experiments/train_target_pivot.py fit
.venv/bin/python experiments/train_target_pivot.py refine
.venv/bin/python experiments/train_target_pivot.py tune
.venv/bin/python experiments/train_target_pivot.py qualify --record-dir experiments/generated/target-pivot-gallery
.venv/bin/python experiments/train_target_pivot.py replay --target 45 --output experiments/generated/pivot-left45.json
.venv/bin/python experiments/train_target_pivot.py replay --target -45 --output experiments/generated/pivot-right45.json
.venv/bin/python experiments/film_target_pivot.py experiments/generated/target-pivot-gallery/nominal-90.json experiments/generated/target-pivot-gallery/nominal--90.json --output experiments/generated/quarter-turns.mp4
.venv/bin/python experiments/film_pivot_suite.py experiments/generated/target-pivot-gallery experiments/generated/all-target-pivots.mp4
```

The [trained policy](../configs/target-edge-pivot.json),
[single-stroke measurements](../experiments/reports/motor-training-target-edge-pivot-fit.json)
and [multi-stroke trials](../experiments/reports/motor-training-target-edge-pivot.json)
contain the learned response tables, physical measurements and individual outcomes.
