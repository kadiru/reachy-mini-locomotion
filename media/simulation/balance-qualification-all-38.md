# Reachy Mini balance tests

Each chapter shows one full simulated trial at real speed, followed by a two-second freeze of its terminal pose. The orange line marks first ball contact. The tilt plot is positive toward the requested edge. The 38 chapter outcomes were checked against the frozen qualification report.

| Start (m:ss) | Test | Edge and condition | Result | Observation |
| --- | ---: | --- | --- | --- |
| 0:00 | 01 | Left edge, nominal conditions, ball from left | PASS | Ball rebounded. Balance recovered in 0.42 s and held through the final 5 s. |
| 0:16 | 02 | Left edge, nominal conditions, ball from right | PASS | Ball rebounded. Balance recovered in 0.52 s and held through the final 5 s. |
| 0:32 | 03 | Left edge, physics step 0.1 ms, ball from left | FAIL | Body tilt reached the 35° stop at 2.37 s, before ball release. |
| 0:36 | 04 | Left edge, physics step 0.1 ms, ball from right | FAIL | Body tilt reached the 35° stop at 2.37 s, before ball release. |
| 0:40 | 05 | Left edge, physics step 0.25 ms, ball from left | FAIL | Body tilt reached the 35° stop at 2.65 s, before ball release. |
| 0:45 | 06 | Left edge, physics step 0.25 ms, ball from right | FAIL | Body tilt reached the 35° stop at 2.65 s, before ball release. |
| 0:50 | 07 | Left edge, fresh sensor noise seed, ball from right | PASS | Ball rebounded. Balance recovered in 0.81 s and held through the final 5 s. |
| 1:06 | 08 | Left edge, attitude noise raised to 0.1°, ball from right | PASS | Ball rebounded. Balance recovered in 0.37 s and held through the final 5 s. |
| 1:22 | 09 | Left edge, gyro noise raised to 0.3°/s, ball from right | FAIL | Body tilt reached the 35° stop at 3.00 s, before ball release. |
| 1:27 | 10 | Left edge, observation delay doubled to 40 ms, ball from right | FAIL | Body tilt reached the 35° stop at 2.43 s, before ball release. |
| 1:31 | 11 | Left edge, robot mass −5%, ball from right | FAIL | Ball rebounded against the upright base. Qualified edge hold lasted 0.00 s. |
| 1:47 | 12 | Left edge, robot mass +5%, ball from right | FAIL | Ball rebounded against the upright base. Qualified edge hold lasted 0.00 s. |
| 2:03 | 13 | Left edge, heavier, faster ball, ball from right | FAIL | Joint limit exceeded by 0.29° at 5.86 s, 0.42 s after ball contact. |
| 2:11 | 14 | Left edge, ball diameter reduced to 6 cm, ball from right | FAIL | Body tilt reached the 35° stop at 6.12 s, 0.31 s after ball contact. |
| 2:19 | 15 | Right edge, nominal conditions, ball from left | PASS | Ball rebounded. Balance recovered in 1.10 s and held through the final 5 s. |
| 2:35 | 16 | Right edge, nominal conditions, ball from right | PASS | Ball rebounded. Balance recovered in 0.38 s and held through the final 5 s. |
| 2:51 | 17 | Right edge, physics step 0.1 ms, ball from left | FAIL | Body tilt reached the 35° stop at 3.54 s, before ball release. |
| 2:57 | 18 | Right edge, physics step 0.1 ms, ball from right | FAIL | Body tilt reached the 35° stop at 3.54 s, before ball release. |
| 3:02 | 19 | Right edge, physics step 0.25 ms, ball from left | FAIL | Body tilt reached the 35° stop at 2.74 s, before ball release. |
| 3:07 | 20 | Right edge, physics step 0.25 ms, ball from right | FAIL | Body tilt reached the 35° stop at 2.74 s, before ball release. |
| 3:12 | 21 | Right edge, fresh sensor noise seed, ball from left | PASS | Ball rebounded. Balance recovered in 1.26 s and held through the final 5 s. |
| 3:28 | 22 | Right edge, attitude noise raised to 0.1°, ball from left | FAIL | Joint limit exceeded by 0.29° at 3.77 s, before ball release. |
| 3:34 | 23 | Right edge, gyro noise raised to 0.3°/s, ball from left | FAIL | Body tilt reached the 35° stop at 2.73 s, before ball release. |
| 3:38 | 24 | Right edge, observation delay doubled to 40 ms, ball from left | FAIL | Body tilt reached the 35° stop at 2.38 s, before ball release. |
| 3:43 | 25 | Right edge, robot mass −5%, ball from left | FAIL | Ball rebounded against the upright base. Qualified edge hold lasted 0.00 s. |
| 3:59 | 26 | Right edge, robot mass +5%, ball from left | FAIL | Ball rebounded against the upright base. Qualified edge hold lasted 0.00 s. |
| 4:15 | 27 | Right edge, heavier, faster ball, ball from left | FAIL | Joint limit exceeded by 0.29° at 5.82 s, 0.39 s after ball contact. |
| 4:23 | 28 | Right edge, ball diameter reduced to 6 cm, ball from left | FAIL | Body tilt reached the 35° stop at 6.10 s, 0.30 s after ball contact. |
| 4:31 | 29 | Left edge, hold and return at 0.125 ms, flat start | PASS | Returned flat. Longest qualifying hold was 5.16 s (required 4.90 s). |
| 4:51 | 30 | Left edge, hold and return at 0.25 ms, flat start | FAIL | Body tilt reached the 35° stop at 2.65 s. |
| 4:55 | 31 | Right edge, hold and return at 0.125 ms, flat start | PASS | Returned flat. Longest qualifying hold was 4.98 s (required 4.90 s). |
| 5:15 | 32 | Right edge, hold and return at 0.25 ms, flat start | FAIL | Body tilt reached the 35° stop at 2.74 s. |
| 5:20 | 33 | Front edge, hold and return at 0.125 ms, raised start at 32.1° | PASS | Returned flat. Longest qualifying hold was 5.18 s (required 4.90 s). |
| 5:40 | 34 | Front edge, hold and return at 0.25 ms, raised start at 32.1° | PASS | Returned flat. Longest qualifying hold was 5.18 s (required 4.90 s). |
| 6:00 | 35 | Back edge, hold and return at 0.125 ms, raised start at 32.1° | PASS | Returned flat. Longest qualifying hold was 5.08 s (required 4.90 s). |
| 6:20 | 36 | Back edge, hold and return at 0.25 ms, raised start at 32.1° | PASS | Returned flat. Longest qualifying hold was 5.04 s (required 4.90 s). |
| 6:40 | 37 | Front edge, moving front catch at 0.125 ms, raised start at 31° · 10°/s outward | FAIL | Returned flat. Longest qualifying hold was 4.42 s (required 4.90 s). |
| 7:00 | 38 | Front edge, moving front catch at 0.25 ms, raised start at 31° · 10°/s outward | PASS | Returned flat. Longest qualifying hold was 4.94 s (required 4.90 s). |
