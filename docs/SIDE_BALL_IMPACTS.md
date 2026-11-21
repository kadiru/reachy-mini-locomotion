# Side impacts during edge balance

## Direct horizontal releases

The direct-hit follow-up used 5 g and 10 g simulated balls released horizontally
on both sides of each balancing edge at 1.0, 1.4 and 2.0 m/s. The release centre
was 18 cm above the floor. The tilted torso projects outward on its loaded side,
so the release distances from the base centre were 30 cm on that side and 21 cm
on the opposite side. Each ball was clear of the robot at release and flew for
33–135 ms before first robot contact. All 24 first hits preceded floor contact.
The frozen policy recovered in 12 trials. All 12 other trials reached a head
joint limit. The reel shows the impact and recovery portion of each successful
episode, with a 0.2× view of the ball in flight. Every included episode passed
the final five-second edge-hold test. [Direct-hit reel](../media/simulation/direct-airborne-recoveries.mp4) · [All 24 outcomes](../experiments/reports/motor-training-direct-airborne-2026-11-20.json)

```sh
study=experiments/generated/direct-airborne-replay
.venv/bin/python experiments/side_ball_study.py --direct-only --work "$study" --report "$study/report.json"
.venv/bin/python experiments/film_successful_side_impacts.py "$study/report.json" --direct-only --states "$study/states" --work "$study/film" --output "$study/recoveries.mp4"
```

## Earlier rolling and airborne comparison

Reachy recovered and held its edge after 10 of 30 verified airborne hits and 3 of 16 rolling hits. All 48 disturbed trials had qualifying edge balance before impact. Two attempted throws bounced on the floor first. One of these recovered and the other reached a joint limit. All four unstruck controls held their edges. The 52 trials evaluate the existing controller in simulation. [Matched comparison video](../media/simulation/side-impact-comparison.mp4) · [All 52 trials](../media/simulation/side-impact-all.mp4) · [Searchable results](../experiments/reports/motor-training-side-impacts-2026-11-20.csv)

| Horizontal release speed | Rolling | Thrown from 12 cm | Thrown from 20 cm |
|---|---:|---:|---:|
| 0.6 m/s | 2/4 | 2/2 airborne | 4/4 |
| 1.2 m/s | 1/4 | 2/4 | 0/4 |
| 2.4 m/s | 0/4 | 0/4 | 0/4 |
| 1.2 m/s, second seed and later release | 0/4 | 2/4 | 0/4 |

Numerators are sustained edge recoveries with a rebound. At 0.6 m/s the two low throws from the raised side reached the floor before the robot and are excluded from the airborne column. In the 1.2 m/s low-throw rows, both recoveries were hits from the raised side. The two supporting-side hits reached a joint limit from each seed. Nine comparable nominal rolling cases reproduced the earlier incoming horizontal speed, termination and final hold.

In the filmed comparison the robot balances on its left edge and the ball comes from its right. The rolling ball strikes the foot at 5.9 cm above the floor at 1.20 m/s and the trial reaches a joint limit. The 12 cm throw also strikes the foot, at 6.6 cm and 1.62 m/s, and the robot recovers. The 20 cm throw strikes the lower torso at 13.3 cm and 1.70 m/s, followed by a joint stop. Identical horizontal release speeds produced different contact heights and velocity vectors.

The largest measured 3-D speed with an airborne recovery was 2.20 m/s. The ball descended onto the raised foot of a right-edge hold with a horizontal component of 0.60 m/s. The qualifying half-second hold began 0.27 s after contact. At a horizontal release of 2.4 m/s, all eight airborne tests reached a joint limit. A rolling hit at that speed toppled the robot on its left edge when the ball came from its right. Across all 48 impacts, including the two floor-first throws, 33 trials reached joint limits. The terminal head-joint side and direction are in the CSV. These observations do not establish a universal toppling speed. [Complete contact vectors and physical outcomes](../experiments/reports/motor-training-side-impacts-2026-11-20.json)

## Experimental setup

The successful-episodes reel shows all 14 contacts followed by a sustained edge recovery. It includes ten airborne hits, three rolling hits, and one throw that bounced off the floor before it hit Reachy. Each chapter follows one episode from its flat start to its verified final hold. Playback slows to 0.35× around contact and returns to real time for recovery. [Watch successful recoveries](../media/simulation/side-impact-recoveries.mp4)

The frozen left and right controllers lift Reachy from flat and hold the requested edge. Each disturbed trial uses one 10 g, 8 cm diameter sphere. We roll it across the floor or release it airborne from a centre height of 12 or 20 cm. The horizontal release speeds are 0.6, 1.2 and 2.4 m/s. We start the sphere 30 cm to the left or right of the base centre, measured in the base heading frame at release. A rolling ball starts in rolling contact with matching spin. A thrown ball starts without spin and with an upward velocity chosen to return to its launch height after 18 cm of horizontal flight. Gravity, collisions and friction then determine its path.

The study uses all four supporting-edge and incoming-side combinations. The primary sweep releases at 5 s. The 1.2 m/s repeat uses a second observation-noise seed and releases at 6 s, so it tests their combined effect. Four unstruck controls check both edges and seeds. Each run lasts up to 14 s with a 0.125 ms physics timestep. The controller uses its existing sensors. The waist holds zero, the nominal battery bus is 5 V, and joint, motor and physical floor-contact criteria apply throughout. Crossing 35° or 40° does not stop a trial. [Policy](../configs/balance-recovery-experimental.json) · [Motor model](../configs/motors-speed-trained.json)

## How to read the results

Release speed refers to the horizontal component. Incoming speed in the film refers to the ball centre's full 3-D speed immediately before first robot contact. The CSV includes both. The height in the film is the measured first contact point above the floor. The contact logger checks when gravity and contact forces first bring a released ball into contact with the floor and robot. A throw counts as an airborne hit only when robot contact precedes floor contact. A ricochet off the floor has its own label.

The edge test checks for a stable pre-impact hold, half a second of continuous strict balance starting after contact and confirmed within three seconds, and a strict hold during the final five seconds. Strict balance requires single-edge support, roll within 2° of the target, roll rate within 5°/s and pitch within 5°. The historical impact pass additionally requires a lateral rebound. The CSV separately reports whether the robot recovered and kept the edge, so the ball's rebound can be evaluated independently. A floor contact by the robot body counts as a topple. A joint-travel stop leaves the later outcome unknown.

## Reproduce the study

From the project root, use the project Python environment and pinned model. Choose fresh output paths. The single-trial command below uses the same 12 cm airborne release and 1.2 m/s horizontal speed as the nominal sweep.

```sh
.venv/bin/python experiments/run_balance_curriculum.py configs/balance-recovery-experimental.json --edge left --mode single-hit --ball-from right --ball-speed 1.2 --launch-height .12 --vertical-speed .73575 --release-at 5 --seconds 14 --record --output experiments/generated/airborne-example.json
study=experiments/generated/side-ball-replay
.venv/bin/python experiments/side_ball_study.py --work "$study" --report "$study/report.json"
.venv/bin/python experiments/review_side_ball_study.py "$study/report.json" --work "$study" --csv "$study/all-trials.csv"
.venv/bin/python experiments/film_successful_side_impacts.py "$study/report.json" --states "$study/states" --work "$study/success-film" --output "$study/recoveries.mp4"
.venv/bin/python experiments/film_hit_envelope.py "$study/highlights.json" "$study/comparison.mp4" --work "$study/comparison-clips" --fps 25 --speed 1
.venv/bin/python experiments/film_hit_envelope.py "$study/chapters.json" "$study/all-tests.mp4" --work "$study/all-clips" --fps 12 --speed 2
```
