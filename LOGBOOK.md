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
| `data/A3/log_gpio_23-10-2026_17-14-58.csv` (**not pushed yet**, on Toghrul's machine in `C:\Users\User\QRCrecords`) | 2026-10-23 17:14 | Early test before R1–R5: still, forward, still, spin, still, spin the other way | Not recorded | unknown (older robot.cpp, no column map) | Original QRC name kept. Toghrul's first A2 notebook read ValueIn0/1 = voltage, 2/3 = wheel speed, 4/5 = wheel angle, 6/7 = accel x/y, 11 = a gyro axis seeing ~5 % of the yaw rate. The A3 notebook takes R (sensor noise) from ValueIn9, which is **inferred from the data** (stable at 0.35 m when still, drop-outs at 24.78 m), not from the source code. At 0.35 m the robot was above the sensor's valid range. |
| `data/A3/S_50mm.csv` | 2026-11-17 13:27 | Static sensor, no button (67.5 s) | Robot square to a flat wall, 50 mm tape from the robot's front edge, robot in its normal (tilted) posture, motors off | `identification_a3` | Hand check before the S series: front sensor reads sensibly from 3 to 30 cm; beyond 30 cm it over-reads (hand at 40 cm read 0.525 m), no jump to 24.78 there. Sensor face is 10 mm behind the front edge. Mean 68.1 mm (bias +18.1 mm), std 0.25 mm, 0 drop-outs. |
| `data/A3/S_100mm.csv` | 2026-11-17 13:37 | Static sensor, no button (45.7 s) | As S_50mm, 100 mm tape | `identification_a3` | Mean 117.3 mm (bias +17.3 mm), std 0.26 mm, 0 drop-outs. |
| `data/A3/S_150mm.csv` | 2026-11-17 13:40 | Static sensor, no button (40.9 s) | As S_50mm, 150 mm tape | `identification_a3` | Mean 171.0 mm (bias +21.0 mm), std 0.95 mm, 0 drop-outs. |
| `data/A3/S_200mm.csv` | 2026-11-17 13:42 | Static sensor, no button (34.2 s) | As S_50mm, 200 mm tape | `identification_a3` | Mean 228.2 mm (bias +28.2 mm), std 0.97 mm, 0 drop-outs. |
| `data/A3/S_250mm.csv` | 2026-11-17 13:44 | Static sensor, no button (30.6 s) | As S_50mm, 250 mm tape | `identification_a3` | Mean 289.4 mm (bias +39.4 mm), std 2.16 mm, 0 drop-outs. |
| `data/A3/S_300mm.csv` | 2026-11-17 13:47 | Static sensor, no button (50.1 s) | As S_50mm, 300 mm tape | `identification_a3` | Mean 370.9 mm (bias +70.9 mm), std 5.07 mm, 0 drop-outs. 99.9 % of the readings are above 0.35 m, so the 0.04–0.35 m gate would reject correct readings at 30 cm. Bias over the S series: about +17 mm up to 100 mm (10 mm of it is the sensor set-back), then growing, so the sensor calibration over-reads with distance. Noise grows about 20x from 50 to 300 mm. |

Battery voltage was not recorded for any of these logs.

## Planned A3 recordings (not recorded yet)

Names follow CLAUDE.md. `<n>` = run number, distances in mm from the ruler.

| File | Firmware | Test | Used for |
|---|---|---|---|
| `S_<dist>mm.csv` | `identification_a3`, no button | Robot still at `<dist>` from the wall (40–300 mm), 15 s | R = sigma_d² (A3 notebook 2.2). Several distances show how the noise grows with distance. |
| `W<n>_start<mm>_end<mm>.csv` | `identification_a3` button 1, or `state_feedback` (estimator only) | Drive or push towards the wall from ≤ 300 mm | Offline replay of the filter (A3 notebook section 5) |
| `M<n>_lifted.csv` | `identification_a3` button 2 | Voltage steps, robot lifted | Motor model (extended A3 case) |
