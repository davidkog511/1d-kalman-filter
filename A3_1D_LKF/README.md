# A3: 1D linear Kalman filter
Design of a linear Kalman filter that estimates the distance from the mobile robot to a wall in front. This estimator will be used inside a control loop that makes sure the robot stays at a fixed distance of a wall in front.

KU Leuven, Mobile Robotics, Assignment 3 (mandatory simple case: the state is only the distance to the wall, the inputs are the two wheel speeds).

## Files

| File | What it is |
|---|---|
| `Assignment3_LKF.ipynb` | Model derivation, noise estimation, simulation of a wall approach, offline replay of a real recording, and a check that its values equal the firmware's |
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
| H | `-R_WHEEL` | the **calibrated** sensor measures `d = -R_WHEEL * x` |
| Calibration | `d = CAL_C2*m^2 + CAL_C1*m + CAL_C0`, m = `getFrontDistance()` | quadratic fit on the static recordings `data/A3/S_50mm` … `S_300mm` (tape distance from the robot's front edge; absorbs the library-curve bias of +17…+71 mm and the sensor mounting offset). Valid 50–300 mm only |
| R | `R(d) = (NOISE_C * dd/dm * (m - 0.0109)^2 / 24.7694)^2 + SIGMA_CAL^2`, evaluated every step | ADC quantisation: one count is worth more distance further away (≈ d²). NOISE_C = 0.83 counts fitted on the S recordings; σ ≈ 0.1 mm at 50 mm, 2.6 mm at 300 mm. SIGMA_CAL = 2 mm: floor for the calibration error (residual rms 1.2 mm, max ~3 mm), treated as white noise although it is systematic (pragmatic approximation); √R ≈ 2.0–3.2 mm |

The front sensor has drop-outs that read 24.78 m. `CorrectionUpdate` therefore has a **gate**:

* on the raw reading m: drop-outs, NaN and inf are skipped, and so is anything above 0.70 m (this also keeps the quadratic one-to-one);
* on the **calibrated** distance: readings outside 0.04–0.32 m are skipped (Sharp GP2Y0A41SK range ~0.04–0.30 m plus margin);
* innovations above 3 sigma (`nu^2 > 9 * S`) are skipped, **unless** 20 samples in a row (0.2 s) fail the test. Then the measurement is accepted again and `_Phat` is first widened to `P_RESYNC` (9.5 rad² ≈ 10 cm), so the filter actually catches up, because the estimate is wrong, not the sensor. The counter is cleared in `resetStateEstimator()`. Without this re-sync a wrong initial guess (e.g. ValueOut1 left at 0) locks the filter out forever. In the simulation the pure 3-sigma gate ends 465 mm off, while the re-sync version ends 5 mm off.

Simulation results: **out of date**, to be updated after the validation on the W recordings (see the notebook for the current simulated values).

## Values to fill in `robot.cpp`

```cpp
K(0) = 2.0;                            // starting guess [1/s]; POSITIVE with the template law (see below), tune it on the robot
_Phat(0,0) = P0_STATE;                 // (0.005 m / R_WHEEL)^2 = 0.0237 rad^2: ruler uncertainty of the start distance (state_estimator.h)
_xhat(0) = readValue(1)*Meas2State;    // initial distance in metres typed in ValueOut1
```

* Template law, unchanged: `desired_velocity = K * (xref - xhat)`. Driving forward increases `x = -d/R`; too far from the wall means `xhat < xref`, so `K > 0` drives forward. Check the sign with the wheels lifted, then raise `K` until the robot is fast but does not oscillate.
* Both inputs are in **metres**: `ValueOut0` = reference distance, `ValueOut1` = initial distance (ruler). The firmware converts them with `Meas2State`.
* Do not change the velocity controller.

Log channels (`writeValue`): 0 uA, 1 uB, 2 posA, 3 posB, 4 spA, 5 spB, 6 xref, 7 front distance (library curve, NOT calibrated) [m], 8 xhat (a priori), 9 xhat (a posteriori), 10 innovation (0 when the reading was rejected), 11 P (a posteriori) [rad^2]. Channels 9–11 are only written while the estimator is on.

## Differences from the course template

`robot.cpp` differs from the course template (Sep 25) only where the assignment asks for values (`K`, `_Phat`, `_xhat`) and in the changes below. Changes to comments only are not listed. `state_estimator.cpp/.h` are ours (the template had only `??`).

| Change | Why |
|---|---|
| Log channels renumbered, P added on 11 | The template writes channels 8 and 9 twice per step, so the a-posteriori estimate and the innovation were overwritten by the wheel angles. Now each signal is logged once. |
| Control only runs when the estimator is on too (`controlEnabled() && StateEstimationEnabled()`) | With the estimator off, `_xhat` is frozen, so the controller would drive on a stale estimate and never stop. |
| `ResetGate()` in `resetStateEstimator()` | The gate's rejected-reading counter lives in `state_estimator.cpp`. Without clearing it, a new run inherits the rejections of the previous one. |
| `_nu(0) = 0` in `resetStateEstimator()` | After a reset the logged innovation starts at 0 instead of the previous run's last value. |
| Button 0 message text | Says that control only starts once the estimator is on (row 2). |
| `speedA`, `speedB` removed from `robot.h` | Unused (never read or written). |

`xref(0) = readValue(0)*Meas2State` is the template line itself, so it is not a difference.

## Tests to run on the robot

1. **Sensor noise.** Robot still about 20 cm from the wall, no buttons, record 15 s. Save as `S_<dist>mm.csv` (e.g. `S_200mm.csv`). Done for 50–300 mm: the notebook (sections 2.2, 2.3) fits the calibration and R(d) on these files.
2. **Estimator only.** Start about 30 cm from the wall (inside the sensor range, 4–30 cm), measure the start distance with the ruler and type it in ValueOut1, press Button 1, push the robot by hand to about 20 cm, record as `W<n>_start<mm>_end<mm>.csv` (start and end distance from the ruler), measure the end distance with a ruler. Replay offline in section 5 of the notebook and compare with channel 9.
3. **Closed loop.** Wheels lifted: check the direction. Floor: type the ruler distance in ValueOut1 and the reference in ValueOut0, press Button 1 then Button 0, tune `K`, record the best run.
4. **Robustness (optional).** Wave a hand in front of the sensor to show the gate works; start with a wrong initial distance to show convergence.

## Instructions for a generative AI assistant (Claude, ChatGPT, ...)

See [`../CLAUDE.md`](../CLAUDE.md).
