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

Battery voltage was not recorded for any of these logs.

## Logs referenced in notebooks but not in the repo yet

| File | Where it is used | Status |
|---|---|---|
| `log_gpio_23-10-2026_17-14-58.csv` | `A3_1D_LKF/Assignment3_LKF.ipynb` (fallback for R = sigma_d², front distance assumed on ValueIn9) | On Toghrul's machine (`C:\Users\User\QRCrecords`). Recorded with an older firmware whose column map is unknown. The notebook infers the front-distance channel from the data. **Toghrul: push it to `data/A3/` and add a row with what was recorded.** |
| `sensor_still.csv` | A3 notebook section 2.2 (test 1 in `A3_1D_LKF/README.md`) | Not recorded yet. |
| `wall_approach.csv` | A3 notebook section 5 (test 2/3) | Not recorded yet. |
