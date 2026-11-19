# Forward progress comparison — November 19, 2026

The user's criticism is correct: the latest smooth demo is much slower than the
original fast demo. It improves steering and sampled acceleration, but is not an
overall improvement in locomotion. The historical videos also used different
measurement durations and different physics. Those differences should have been
foregrounded in the comparison.

Ran 54 diagnostic simulations: three frozen gaits × nine model variants × two
timesteps. No new policy was trained or selected, and production motor limits
were not increased. Each comparison uses exactly **6.6 measured seconds** from
50 Hz saved states, following two excluded warm-up cycles. Gaits run to the next
whole-cycle boundary; the comparison window is clipped at exactly 6.6 s. This
avoids comparing displacement from 6.6 s against displacement from 13 s.

| Model and gait | Forward at 2 ms | Forward at 1 ms | Heading change at 2 / 1 ms |
|---|---:|---:|---:|
| Historical model, 1.2 Nm, fast gait | 158.4 mm | 111.4 mm | -3.8° / -6.9° |
| Same historical model/gait, only cap changed to 0.6 Nm | 2.16 mm | 0.72 mm | +1.9° / +0.9° |
| Current 5 V model, initial trained gait | 68.5 mm | 66.6 mm | -57.1° / -55.8° |
| Current 5 V model, smooth trained gait | 17.7 mm | 16.7 mm | -0.3° / -1.0° |

The old fast result depends heavily on its assumed torque cap **within the old
model**, which also has unrealistic passive-joint losses/inertia. This does not
prove the physical robot cannot reproduce the observed walking mechanism. The
15.8 cm result also drops to 11.1 cm when the timestep is halved: its magnitude
was not converged.

The training-objective tradeoff can be isolated within the newer model: the
initial trained gait moves about four times farther than the smooth gait, but
its lateral drift is 49–50 mm and it turns about 56–57°. The smooth gait's lateral
drift is 1.4–1.9 mm. Favoring heading and acceleration too strongly suppressed
forward progress, a priority the user wants preserved.

## Other diagnostic findings

- Restoring only the lower CAD mass (1.17652824 kg versus the documented 1.475 kg)
  increases the smooth gait's travel to 48–51 mm. The initial trained gait reaches
  210–214 mm, but its heading changes sharply with timestep. This is sensitivity
  evidence, not permission to use a lighter robot to make the demo look better.
- Raising 5 V torque/PWM bounds from our 80% design choice to the full momentary
  datasheet values does not restore the historical fast gait. The smooth gait
  still travels 17 mm; the faster trained gait travels 81–89 mm with large turns.
- Using the current 6 V model gives the smooth gait 23–24 mm forward travel.
  Full momentary bounds at 6 V make the initial trained gait spin around and end
  behind its starting forward axis. Higher torque is not a general solution.
- Relaxing only command speed/acceleration from 6 rad/s and 60 rad/s² to 8 and
  120 reduces the smooth gait to roughly 1 mm. It was exploiting a particular
  timing/phase response; larger command limits alone do not recover speed.

These are model sensitivity experiments. The battery-powered reference video
remains evidence of a real mechanism that the simulator has not been calibrated
to reproduce. Published motor endpoints constrain the model, but do not identify
its servo gain, phase lag, passive friction, mass distribution or foot contact.

## Consequence for the next training pass

Retain the faster learned gait as a development baseline, and optimize forward
speed subject to explicit drift, heading and fall constraints. Treat smoothness
as a bounded requirement or secondary objective. Do not silently increase torque
or reduce mass to recover the old visual. Fit battery-powered actuator response
and contact/mass distribution against measurements before claiming transfer.
No production policy was changed in this diagnostic pass.

[Complete receipt](../experiments/reports/motor-training-forward-comparison-2026-11-19.json)
retains all configs, raw whole-cycle metrics, and 10 Hz paths from each 6.6 s
window. Acceleration in this receipt is a position finite-difference diagnostic;
it is not identical to the velocity-based metric in the previous training report.

```sh
.venv/bin/python experiments/compare_forward_progress.py --output experiments/reports/motor-training-forward-comparison-next.json
```

The first diagnostic attempt stopped at a mass-validation guard because a rounded
CAD mass was infinitesimally below the XML sum. The script now reads the exact
sum from the pinned XML, saves incremental results, and the full rerun completed.
No physics validation threshold was weakened.
