# Simultaneous overhead drops during side balance

**Two balls were enough to lose the edge. Four produced the first confirmed topple.** In the filmed left-edge trial, all four balls struck the head within 16 ms. The lower torso touched the floor 1.92 s after the first ball hit, at 70.9° tilt. Reachy tipped as far as 91.6° and came to rest at 72.9°. Physics continued for another 6.89 s after ground contact. At this release time, one ball allowed Reachy to recover its left edge. [Side-by-side replay at half speed](../media/simulation/overhead-drop-fall.mp4)

### Outcomes by number of balls

For the matched sweep below, Reachy met the full pre-drop balance test on both sides. Two sensor and release-time conditions were used, at 6.0 and 6.5 s. In each of these 40 disturbed trials **every released ball struck the robot before the floor**. Four unstruck controls held their edges. Counts 8–10 use the compact arrangement described below.

| Balls dropped together | Held original edge | Settled flat on base | Other finish |
|---:|---:|---:|---|
| 1 | 4/4 | 0/4 | |
| 2 | 0/4 | 4/4 | |
| 3 | 1/4 | 3/4 | |
| 4 | 2/4 | 2/4 | |
| 5 | 2/4 | 1/4 | One missed both final hold and flat-settling criteria |
| 6 | 2/4 | 2/4 | |
| 7 | 2/4 | 2/4 | |
| 8 | 0/4 | 4/4 | |
| 9 | 3/4 | 1/4 | |
| 10 | 2/4 | 2/4 | Both right-edge trials held and both left-edge trials settled flat |

Across all conditions, three trials produced actual body contact with the floor. The four-ball left-edge fall occurred with a 5.0 s release. On the right edge, the wider nine-ball arrangement produced a fall at each of the later release times, with eight of nine balls hitting the robot. Both finished around 72.8° tilt. The compact nine-ball arrangement gave right-edge holds at both release times. Ball count alone cannot determine the outcome. The contact times and locations also vary as balls strike the tilted shell and collide with one another.

The archive covers 78 episodes, including six unstruck controls. The right-edge 5.0 s release missed the required pre-hold by one 20 ms sample, so its ten drops and one control are labeled unqualified. They are included in the film and excluded from the matched comparison. For ten direct hits on each side, jump to 3:04 or 3:19. The CSV gives the chapter start time, actual hits, first floor contact, final orientation, and joint warnings for every episode. [Complete chaptered film](../media/simulation/overhead-drops-all.mp4) · [Trial index and outcomes](../experiments/reports/motor-training-overhead-summary-2026-11-20.csv)

## Method

Each episode starts with Reachy flat. Its existing controller raises one side and balances on the requested rim. We release 1 through 10 balls together from rest in a single horizontal layer above the head, plus an unstruck control. Each ball is 3 cm in diameter and weighs 10 g. Its centre starts 48 cm above the floor. Gravity, ball–ball contact and ball–robot contact determine when each ball arrives. The balls have 32 mm spacing between nearest neighbours at release. The same ten sphere bodies are present in every simulation. Unused balls remain suspended away from the robot.

The simulations use a 0.125 ms physics step, the existing 5 V motor envelope, a zero waist target and the frozen balance policy. The policy gets its usual sensor inputs. It receives no ball state or advance notification. The study runs to 14 simulated seconds with no tilt cutoff. Software joint-travel warnings are logged while MuJoCo's physical joint constraints and servo limits remain active. Numerical and mechanism-closure checks remain active. Joint constraints in this model are compliant, so some measured positions briefly exceed their nominal limits.

A fall requires measured non-foot robot-body contact with the floor above 0.5 N. The experiment records which link touched, time, and tilt at that instant and then continues integrating. Settling flat on the base is a separate outcome. A successful edge result requires the full final five seconds within 2° of target roll, roll rate within 5°/s, pitch within 5°, and single-edge support. Prequalification requires the same strict hold for a full second immediately before release.

## Reproduce

Run these commands in the project environment with the pinned model installed. Each study uses a fresh directory, preserves physics trajectories, and counts how many released balls actually struck Reachy before reaching the floor. The compact pattern places the final three balls across the wider axis of the head. Counts one through seven use identical positions in both layouts.

```sh
work=experiments/generated/drop-replay
mkdir -p "$work"
.venv/bin/python experiments/overhead_drops.py --work "$work/first" --report "$work/first.json" --repeat
.venv/bin/python experiments/overhead_drops.py --work "$work/later" --report "$work/later.json" --nominal-release 6.5
.venv/bin/python experiments/overhead_drops.py --work "$work/compact" --report "$work/compact.json" --nominal-release 6.5 --repeat --counts 8 9 10 --compact
.venv/bin/python experiments/summarize_overhead_drops.py "$work/first.json" "$work/later.json" "$work/compact.json" --csv "$work/trials.csv"
.venv/bin/python experiments/film_overhead_drops.py --example "$work/first/states/left-01-nominal.json" "$work/first/states/left-04-nominal.json" --work "$work/example-clips" --output "$work/example.mp4"
.venv/bin/python experiments/film_overhead_drops.py --study "$work/first.json" "$work/first/states" --study "$work/later.json" "$work/later/states" --study "$work/compact.json" "$work/compact/states" --work "$work/all-clips" --output "$work/all.mp4"
```
