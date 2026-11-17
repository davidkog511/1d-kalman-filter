# Logbook

One row per recorded log. Add the row in the same commit as the CSV.
*Firmware* is the `load_log(..., firmware=...)` key in `mr_common.py`, i.e. which `robot.cpp` wrote the log.
Date and time come from the first line of the CSV (QRoboticsCenter header). All logs are 100 Hz (10 ms).

| File | Date | Test | Setup (distances, voltage) | Firmware | Notes |
|---|---|---|---|---|---|
| `data/A2/R1_static.csv` | 2026-11-03 14:38 | Robot still, no button (80.7 s) | Flat floor, motors off (0 V) | `identification` | IMU bias and noise, Q of the A2 IMU model. az reads 8.82 instead of 9.81 m/s² (z scale or offset error, outside datasheet tolerance). |
| `data/A2/R2_front_up.csv` | 2026-11-03 14:43 | Front side lifted by hand (15.7 s) | Motors off | `identification` | Meant for the IMU orientation. The tilt was too small to tell apart from the floor unevenness, so it is not used in the notebook. |
| `data/A2/R3_left_up.csv` | 2026-11-03 14:45 | Left side lifted by hand (16.6 s) | Motors off | `identification` | Used for the IMU mounting yaw (PSI). |
| `data/A2/R4_straight_fwd700mm_right190mm_turnR15deg.csv` | 2026-11-03 14:48 | Button 1: straight drive (53.3 s) | 30 s still, then 2 s at +6 V on both motors. End position by tape measure: 700 mm forward, 190 mm right. | `identification` | The 15° heading in the file name is **wrong**. From the end position the heading is about −30°. Slip and yaw at 32.5–32.9 s. Static window 5–29.5 s. Source of sigma_v and Q1. |
| `data/A2/R5_rotation_CW.csv` | 2026-11-03 14:55 | Button 2: rotation in place, clockwise (25.6 s) | 5 s still, 8 s at ±4 V, 8 s at ±8 V (A forward, B backward) | `identification` | Lever arm R_ARM and gyro tilt. The wheels over-count the yaw rate by about 7 % (skid). |
| `data/A3/log_gpio_23-10-2026_17-14-58.csv` (**not pushed yet**, on Toghrul's machine in `C:\Users\User\QRCrecords`) | 2026-10-23 17:14 | Early test before R1–R5: still, forward, still, spin, still, spin the other way | Not recorded | unknown (older robot.cpp, no column map) | Original QRC name kept. Toghrul's first A2 notebook read ValueIn0/1 = voltage, 2/3 = wheel speed, 4/5 = wheel angle, 6/7 = accel x/y, 11 = a gyro axis seeing ~5 % of the yaw rate. Front distance probably on ValueIn9 (**inferred from the data**: stable at 0.35 m when still, drop-outs at 24.78 m). **No longer used**: the calibration and R(d) now come from the S recordings. |
| `data/A3/S_50mm.csv` | 2026-11-17 13:27 | Static sensor, no button (67.5 s) | Robot square to a flat wall, 50 mm tape from the robot's front edge, robot in its normal (tilted) posture, motors off | `identification_a3` | Hand check before the S series: front sensor reads sensibly from 3 to 30 cm; beyond 30 cm it over-reads (hand at 40 cm read 0.525 m), no jump to 24.78 there. Sensor face is 10 mm behind the front edge. Mean 68.1 mm (bias +18.1 mm), std 0.25 mm, 0 drop-outs. |
| `data/A3/S_100mm.csv` | 2026-11-17 13:37 | Static sensor, no button (45.7 s) | As S_50mm, 100 mm tape | `identification_a3` | Mean 117.3 mm (bias +17.3 mm), std 0.26 mm, 0 drop-outs. |
| `data/A3/S_150mm.csv` | 2026-11-17 13:40 | Static sensor, no button (40.9 s) | As S_50mm, 150 mm tape | `identification_a3` | Mean 171.0 mm (bias +21.0 mm), std 0.95 mm, 0 drop-outs. |
| `data/A3/S_200mm.csv` | 2026-11-17 13:42 | Static sensor, no button (34.2 s) | As S_50mm, 200 mm tape | `identification_a3` | Mean 228.2 mm (bias +28.2 mm), std 0.97 mm, 0 drop-outs. |
| `data/A3/S_250mm.csv` | 2026-11-17 13:44 | Static sensor, no button (30.6 s) | As S_50mm, 250 mm tape | `identification_a3` | Mean 289.4 mm (bias +39.4 mm), std 2.16 mm, 0 drop-outs. |
| `data/A3/S_300mm.csv` | 2026-11-17 13:47 | Static sensor, no button (50.1 s) | As S_50mm, 300 mm tape | `identification_a3` | Mean 370.9 mm (bias +70.9 mm), std 5.07 mm, 0 drop-outs. 99.9 % of the readings are above 0.35 m, so the 0.04–0.35 m gate would reject correct readings at 30 cm. Bias over the S series: about +17 mm up to 100 mm (10 mm of it is the sensor set-back), then growing, so the sensor calibration over-reads with distance. Noise grows about 20x from 50 to 300 mm. |
| `data/A3/W1_start300_end44.csv` | 2026-11-17 13:52 | Button 1: drive to the wall (34.7 s) | Start 300 mm, end 44 mm (tape from the front edge). 14.8 s still, then 1.21 s at 3.5 V on both motors; stopped by the firmware (sensor < 80 mm) and coasted. | `identification_a3` | Wheel distance 254.5 mm vs tape 256 mm (−1.5 mm). Heading change −1.3° (straight). 0 drop-outs. Sensor − tape: +56 mm at the start, +26 mm at the end. Ended just below the planned 50 mm because of coasting (sensor face 54 mm from the wall). |
| `data/A3/W2_start300_end45.csv` | 2026-11-17 14:00 | Button 1: drive to the wall (24.5 s) | Start 300 mm, end 45 mm. 15.4 s still, 1.17 s at 3.5 V. | `identification_a3` | QRC file was saved as `W2_star300_end45` (typo); stored here with the correct name. Wheel distance 249.2 mm vs tape 255 mm (−5.8 mm). Heading −1.5°. 0 drop-outs. Sensor − tape: +51 mm start, +24 mm end. |
| `data/A3/W3_start300_end42.csv` | 2026-11-17 14:04 | Button 1: drive to the wall (18.0 s) | Start 300 mm, end 42 mm. 12.6 s still, 1.18 s at 3.5 V. | `identification_a3` | Wheel distance 248.8 mm vs tape 258 mm (−9.2 mm). Heading +2.9° (wheel B faster this time). 0 drop-outs. Sensor − tape: +51 mm start, +28 mm end. Over W1–W3 the wheel odometry is short by 0.6–3.6 %: a scale error, not white noise. |
| `data/A3/M1_lifted.csv` | 2026-11-17 14:09 | Button 2: voltage steps, robot lifted (28.1 s) | Wheels free in the air. Steps of 1.5 s: 0, 3, 6, 9, 6, 3, 0, −3, −6, −9, −3, 0 V (test starts at t = 3.07 s) | `identification_a3` | All 12 steps visible, VA = VB within 0.04 V of the setpoint. No dead zone (6 rad/s at 3 V). Steady speed about 2.2 rad/s per V with about 0.2 V offset, symmetric for + and −; wheel A about 1 % faster than B. Currents: spikes of 0.1–0.2 A at each step, steady 0.01–0.03 when lifted. |

Battery voltage was not recorded for any of these logs.

**A3 recording session 2026-11-17 (applies to all S, W and M files above), observations that matter for the filter:**
- Hand check of the front sensor: readings are sensible from 3 to 30 cm. Beyond 30 cm it **over-reads** (hand at 40 cm read 0.525 m) instead of dropping out to 24.78 m.
- The sensor face is **10 mm behind the robot's front edge**. All tape distances are measured horizontally from the front edge.
- The robot was recorded in its **normal tilted posture** (not levelled), square to a flat wall.
- W runs: the firmware stops driving when the sensor reads < 80 mm; the robot then **coasts to 42–45 mm** (tape, front edge).

## Planned A3 recordings (not recorded yet)

The S series (50–300 mm) is done, see above.

Names follow CLAUDE.md. `<n>` = run number, distances in mm from the ruler.

| File | Firmware | Test | Used for |
|---|---|---|---|
| `W<n>_start<mm>_end<mm>.csv` | `identification_a3` button 1, or `state_feedback` (estimator only) | Drive or push towards the wall from ≤ 300 mm | Offline replay of the filter (A3 notebook section 5) |
| `M<n>_lifted.csv` | `identification_a3` button 2 | Voltage steps, robot lifted | Motor model (extended A3 case) |
