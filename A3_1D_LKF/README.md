# A3: 1D linear Kalman filter
Design of a linear Kalman filter that estimates the distance from the mobile robot to a wall in front. This estimator will be used inside a control loop that makes sure the robot stays at a fixed distance of a wall in front.

KU Leuven, Mobile Robotics, Assignment 3 (mandatory simple case: the state is only the distance to the wall, the inputs are the two wheel speeds).

## Files

| File | What it is |
|---|---|
| `Assignment3_LKF.ipynb` | Model derivation, noise estimation, simulation of a wall approach, offline replay of a real recording, and generation of the C++ file |
| `INTERNAL-StateEstimation_StateFeedback/` | The Arduino sketch: template with `robot.cpp` and `state_estimator.cpp` completed. Compiles for the Arduino Mega (`arduino:avr:mega`) with the course libraries (now in `../libraries/`); not yet run on the robot |
| `../A2_prediction_model/Assignment2_solution.ipynb` | Assignment 2 prediction models (wheel odometry 1D, IMU 2D). Section 2.3 adds the coupling of heading uncertainty into the x, y covariance (EKF Jacobian of the rotation; the mean is unchanged), following the feedback on the submission. The 1D filter uses scenario 1 only, which is not affected. |

The notebook reads local recordings (Assignment 2 logs and QRoboticsCenter CSV files) that are **not** in this repo. Edit the paths and channel names in its "Settings" cell.

## The model

The template's `robot.h` defines `Meas2State = -1/R_WHEEL`, so the state is the **wheel angle relative to the wall**, `x = -d / R_WHEEL` [rad]. `R_WHEEL = 0.0325 m`, `TSAMPLE = 0.01 s`.

| Matrix | Value | Where it comes from |
|---|---|---|
| F | 1 | the wall does not move |
| B | `TSAMPLE`, applied to `v = (wA + wB)/2` [rad/s] | wheel odometry, the prediction model of Assignment 2 (scenario 1) |
| Q | `(TSAMPLE * sigma_omega)^2` = 3.3e-6 rad^2 | `sigma_omega = sigma_v / R_WHEEL`, `sigma_v` = 5.9 mm/s = spread of the wheel speed at constant voltage (Assignment 2, run R4) |
| H | `-R_WHEEL` | the sensor measures `d = -R_WHEEL * x` |
| R | `sigma_d^2` = 4.75e-4 m^2 (21.8 mm) | standard deviation of the front sensor while the robot stands still |

The front sensor has drop-outs that read 24.78 m. `CorrectionUpdate` therefore has a **gate**:

* readings outside 0.04–0.35 m are always skipped (Sharp GP2Y0A41SK range ~0.04–0.30 m plus margin; also catches drop-outs, inf and NaN);
* innovations above 3 sigma (`nu^2 > 9 * S`) are skipped, **unless** 20 samples in a row (0.2 s) fail the test. Then the measurement is accepted again and `_Phat` is reset to `P0_STATE`, so the filter actually catches up, because the estimate is wrong, not the sensor. The counter is cleared in `resetStateEstimator()`. Without this re-sync a wrong initial guess (e.g. ValueOut1 left at 0) locks the filter out forever. In the simulation the pure 3-sigma gate ends 465 mm off, while the re-sync version ends 5 mm off.

Simulation results (simulated approach from 0.45 m to about 0.18 m, noise from the real recordings): about 6 mm RMSE for the filter, 22 mm for the sensor alone, 91 mm for the prediction alone. Without the gate a single drop-out moves the estimate by about 125 mm.

## Values to fill in `robot.cpp`

```cpp
K(0) = 2.0;                            // starting guess [1/s]; POSITIVE with the template law (see below), tune it on the robot
_Phat(0,0) = P0_STATE;                 // 9.5 rad^2 = (0.10 m / R_WHEEL)^2 = 10 cm initial uncertainty (state_estimator.h)
_xhat(0) = readValue(1)*Meas2State;    // initial distance in metres typed in ValueOut1
```

* Template law, unchanged: `desired_velocity = K * (xref - xhat)`. Driving forward increases `x = -d/R`; too far from the wall means `xhat < xref`, so `K > 0` drives forward. Check the sign with the wheels lifted, then raise `K` until the robot is fast but does not oscillate.
* Both inputs are in **metres**: `ValueOut0` = reference distance, `ValueOut1` = initial distance (ruler). The firmware converts them with `Meas2State`.
* Do not change the velocity controller.

Template log channels (`writeValue`): 0 uA, 1 uB, 2 posA, 3 posB, 4 spA, 5 spB, 6 xref, 7 front distance, 8 xhat (a priori), 9 xhat (a posteriori), 10 innovation (0 when the reading was rejected), 11 P (a posteriori) [rad^2]. Channels 9–11 are only written while the estimator is on.

## Tests to run on the robot

1. **Sensor noise.** Robot still about 20 cm from the wall, no buttons, record 15 s. Save as `sensor_still.csv`. Recompute `sigma_d` from channel 7 (the notebook does this automatically if the file exists).
2. **Estimator only.** Start about 45 cm from the wall, type the ruler distance in ValueOut1, press Button 1, push the robot by hand to about 20 cm, record as `wall_approach.csv`, measure the end distance with a ruler. Replay offline in section 5 of the notebook and compare with channel 9.
3. **Closed loop.** Wheels lifted: check the direction. Floor: type the ruler distance in ValueOut1 and the reference in ValueOut0, press Button 1 then Button 0, tune `K`, record the best run.
4. **Robustness (optional).** Wave a hand in front of the sensor to show the gate works; start with a wrong initial distance to show convergence.

## Instructions for a generative AI assistant (Claude, ChatGPT, ...)

Follow these steps in this order. Do not skip the verification steps.

1. **Read the assignment first.** Read the assignment notebook and the template (`robot.cpp`, `robot.h`, `state_estimator.cpp`, `state_estimator.h`). Find out which unit the state uses (look for `Meas2State`), which channels are logged (`writeValue`) and where the `??` placeholders are. Do not guess these from memory.
2. **Reuse the prediction model of Assignment 2** (wheel odometry, 1D): `F = 1`, `B = dt`, input = mean wheel speed, `Q` from the speed noise at constant voltage. Convert it to the state unit of the template (rad) and note the sign: forward driving increases `x = -d/R_WHEEL`.
3. **Measure the noise from real data**, do not invent it: `sigma_v` from the steady parts of a constant-voltage drive, `sigma_d` from a stationary part of a recording. Inspect the raw CSV to find which column is the front distance and check it (stable when still, drop-outs at 24.78 m). If the channel is inferred from data and not from source code, say so.
4. **Build the notebook**: model, noise estimation, a simulated wall approach (sensor noise from the data, drop-outs, a few percent unknown wheel-speed scale error, wrong initial guess), comparison filter vs. sensor vs. prediction only, the effect of the innovation gate, a consistency check (normalised innovation squared should have mean about 1), and a sensitivity check on scaling `Q` and `R`. State clearly that the wall approach is simulated until a real recording exists.
5. **Generate the C++** from the notebook values and check that the arithmetic is consistent with the template (`~H`, `~F`, `Inverse(S)`, the gate before the Kalman gain).
6. **Fill in `robot.cpp`** only at the `??` placeholders (`K`, `_Phat`, `_xhat`). Do not touch the velocity controller. Explain the sign of `K`.
7. **Guide the user through the tests above** one at a time, and ask for the recordings before claiming anything works. Replay each real recording through the same filter offline and compare with the on-board estimate (channel 9).
8. **Be honest about limits**: the code is not compiled until the user compiles it, the simulated results are not robot results, `R` was measured at one distance only (the sensor noise grows with distance), and the wheel-speed error is partly a bias that white-noise `Q` does not represent (a larger `Q` may help; check with the innovation test).

### Ready-to-paste prompt

```
I am working on a KU Leuven Mobile Robotics assignment: a 1D linear Kalman filter for the distance from a MECOtron Arduino robot to a wall. Read README.md in this repo and follow the "Instructions for a generative AI assistant" section step by step. My assignment files (Assignment_1D_LKF.ipynb and the template robot.cpp, robot.h, state_estimator.cpp, state_estimator.h) are in: <PATH>. My recordings are in: <PATH>. Start by reading the template files and tell me what you found before changing anything.
```
