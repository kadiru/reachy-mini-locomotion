# Motor limits and what is still unknown

Checked November 18, 2026. This model is **datasheet-bounded simulation**, not an
identified digital twin, a continuous-duty rating, or a hardware-ready policy.

[Reachy's hardware sheet](https://huggingface.co/docs/reachy_mini/en/platforms/reachy_mini/hardware)
identifies six XL330-M288-T head motors, two XL330-M077-T antenna motors, and a
custom XC330-M288-PG body motor (plastic gears). It lists 1.475 kg total mass.
The pinned CAD model sums to 1.17652824 kg. The new configuration corrects total
mass, testing both torso-only and proportional mass/inertia placement. Neither
placement is measured; the missing 298.5 g is not asserted to be a specific part.

| Motor | Stall torque at 5 / 6 V | No-load speed at 5 / 6 V |
|---|---:|---:|
| Head XL330-M288-T | 0.52 / 0.60 Nm | 103 / 123 rpm |
| Antenna XL330-M077-T | 0.215 / 0.228 Nm | 383 / 456 rpm |
| Body XC330-M288-T reference | 0.93 / 1.10 Nm | 81 / 97 rpm |

Sources: [ROBOTIS head](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/),
[antenna](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/),
[body reference](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/).
The custom PG gear train has no independently verified envelope here; body yaw
commands remain zero. The previous 1.2 Nm head assumption exceeded even the
published 6 V stall value by 2×. Historical reports remain reproducible and
explicitly uncalibrated; they are not evidence for real-robot performance.

## Voltage and firmware matter

The manufacturer motor data cover 3.7–6 V. Reachy's system supply is listed as
6.8–7.6 V; [Pollen's troubleshooting guide](https://github.com/pollen-robotics/reachy_mini/blob/main/docs/source/troubleshooting.md)
says its higher voltage is intentional. The user confirmed battery operation for the reference video. Neither source establishes this user's
motor-bus voltage under load. We interpolate only within manufacturer data and
refuse extrapolation. Nominal 5 V and randomized 4.2–6 V are conservative study
hypotheses, not readings from the robot. No power or firmware settings were changed.

The [pinned hardware configuration](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/assets/config/hardware_config.yaml)
uses position mode 3; P/I/D registers 300/0/0 on the head and 200/0/0 on body and
antennas. Head limits (degrees) are [-48,80], [-80,70], [-48,80], [-80,48],
[-70,80], [-80,48]. These match the pinned MJCF. IK solutions outside these
limits are rejected. The [backend](https://github.com/pollen-robotics/reachy_mini/blob/a9a2a42adcc1abc7d9c90d390889bbebf930c96c/src/reachy_mini/daemon/backend/robot/backend.py)
uses 50 Hz commands; installed versions and actual registers remain unknown.

The head manual lists factory current limit 1750 mA and velocity-limit register
445 × 0.229 rpm. These are **not universal position-mode output clamps**:
current limiting applies to current/current-based position modes; Goal Velocity
is not a position-mode speed limit. Profile Velocity and Profile Acceleration
have separate semantics, and zero means unbounded in velocity-based profiles.
We do not silently treat register defaults as installed settings.

## Implemented approximation

`configs/motors-datasheet.json` selects `motors.py`. The reset still uses the
legacy position servo to close the mechanism; before evaluation it is replaced
with torque actuators, then settled for two seconds. No base repositioning or
external force occurs in a rollout. Old configurations retain their old physics.

For each motor, the model clips inferred position-controller PWM, applies a
linear DC back-EMF approximation, and clips torque:

```
pwm = clip(k * (filtered_target - q), -pwm_limit, pwm_limit)
torque = clip(stall_torque * (pwm - velocity / no_load_speed), -cap, cap)
```

The count-to-PWM mapping uses `(P/128)*(4096/(2*pi))/885`; it is an **inference**,
not an identified gain in Nm/rad. Training varies that gain. Back-driving above
no-load speed is physically allowed; simulation never edits velocity to enforce
an artificial speed ceiling. Stall torque is momentary, not a continuous rating.
Nominal torque and PWM limits of 80%, command speed 6 rad/s, acceleration 60
rad/s², a 5 Hz target filter, and one 20 ms delay frame are **our software/model
choices**, not ROBOTIS ratings or changes to Reachy's firmware.

The CAD exporter applies a motor class to seven passive ball joints, including
0.001 kg·m² reflected rotor inertia at each axis. We remove that motor inertia
and explicitly assume small passive damping/friction, varying friction in the
study. Active motor friction, armature and damping remain estimates. No thermal
model, gearbox backlash, electrical inductance or supply-sag model is claimed.
Torque RMS/peak and PWM saturation are recorded; simulated current is not
presented as measured current. The SDK's gravity-compensation code itself notes
uncertainty in its current conversion.

## Before transferring a policy

Obtain read-only motor identity/firmware and registers: operating/drive modes,
P/I/D, PWM/current/velocity limits, Goal PWM/Current, profile velocity/acceleration,
angle limits and offsets, plus voltage, temperature, measured position/velocity
and current during an existing gentle motion. Do not run motor-setup scripts:
they write configuration. Identify motor response and passive friction, weigh
the robot, and measure underside contact/COM. Replay acceptance remains behind
the existing hardware commissioning guard; this optimizer cannot command a robot.
