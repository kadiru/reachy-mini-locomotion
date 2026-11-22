# Supported Reachy Mini tilt calibration

The first hardware session compares real foot contact and sensor readings with the CAD release experiment. Keep the wireless Reachy on battery and support the torso throughout. The recorder polls the daemon. The operator sets each pose and keeps the head neutral for the three tilt recordings.

1. Place the robot upright on the intended floor. Frame the lower shell and the floor from directly in front, with a ruler in the plane of motion. A second side view or a view of the contact patch from underneath helps locate fore–aft contact. Record the floor material and whether the real foot has pads or other attachments. Keep the framing fixed.
2. Record five seconds upright. Support the torso, tilt toward the left until an edge is visibly loaded, hold it still, and record. Repeat on the right at approximately the same angle. A moderate tilt is sufficient. Record the actual angle from video. Keep fingers clear of the neck linkage. A still photo from each pose with the scale visible is enough for the geometry comparison.
3. With the base upright and supported, optionally use the usual operator controls to make a small head roll each way while recording the supported-head clip. The recording script issues only HTTP GET requests. It logs the daemon's target and measured pose. Keep the raw video and identify the head movement in both recordings.

From the project directory, set the daemon origin and the actual surface name. Use a new output directory for every session. Each capture starts **after** the operator has set the corresponding pose. The recorder refuses to overwrite an earlier take.

```sh
REACHY_ORIGIN=http://reachy-mini.local:8000
SURFACE='wood floor'
RUN=data/physical/supported-tilt-01
.venv/bin/python experiments/supported_tilt.py capture upright --url "$REACHY_ORIGIN" --surface "$SURFACE" --output "$RUN/upright.jsonl"
.venv/bin/python experiments/supported_tilt.py capture left --url "$REACHY_ORIGIN" --surface "$SURFACE" --output "$RUN/left.jsonl"
.venv/bin/python experiments/supported_tilt.py capture right --url "$REACHY_ORIGIN" --surface "$SURFACE" --output "$RUN/right.jsonl"
.venv/bin/python experiments/supported_tilt.py compare "$RUN/upright.jsonl" "$RUN/left.jsonl" "$RUN/right.jsonl"
```

The comparison reports the angle between median accelerometer vectors, gyro activity during each hold, target-versus-present head orientation and lateral error, HTTP timing, and valid sample counts. The angle is unsigned. Its interpretation as foot tilt requires a stationary head relative to the base, a stationary robot, and agreement with the independent camera measurement. If IMU data are missing the angle is reported as null. HTTP timing measures request to host receipt. The pinned wireless daemon updates its BMI088 cache in the backend loop, documented at 50 Hz. Our controller averages simulated gyro measurements at 1 kHz and acts at 50 Hz. A 25 Hz GET recording cannot establish that these signals have the same delay or bandwidth. Calibrating sensor age or motor response delay requires synchronized motion observations. A side photo can identify a contact region. The force-weighted center of pressure in the simulation would require a pressure measurement for a direct physical comparison.

## Predictions to check against the supported robot

With the foot rigidly supported and the head commanded neutral, the 5 V simulation predicts the following settled head offsets. The support holds the foot angle throughout. These values give us a direct check on simulated servo stiffness before an unsupported catch.

| Supported roll | Measured head roll | Measured lateral head shift |
|---:|---:|---:|
| Upright | +0.17° | +0.09 mm |
| Left 20° | −1.38° | +2.71 mm |
| Right 20° | +1.69° | −2.55 mm |
| Left 32.2° | −2.22° | +4.14 mm |
| Right 32.2° | +2.48° | −4.01 mm |

Use the angle observed on camera to run the corresponding fixed-base comparison, for example `.venv/bin/python experiments/supported_tilt_model.py -18.5 21.0`. Compare changes from the upright reading and confirm camera, SDK and MJCF axis signs before interpreting a signed discrepancy. In the model, the head frame is the MJCF head site relative to the foot. The recorder reports target-relative orientation and lateral displacement in the daemon frame.
