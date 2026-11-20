# A3: 1D linear Kalman filter
Design of a linear Kalman filter that estimates the distance from the mobile robot to a wall in front. This estimator will be used inside a control loop that makes sure the robot stays at a fixed distance of a wall in front.

KU Leuven, Mobile Robotics, Assignment 3 (mandatory simple case: the state is only the distance to the wall, the inputs are the two wheel speeds).

## Files

| File | What it is |
|---|---|
| `Assignment3_LKF.ipynb` | **Main notebook (for submission):** design table, prediction model, sensor calibration, R(d), Q(ω), filter and gate, held-out validation on W1–W7, firmware consistency check, closed-loop tests C1–C5, conclusions |
| `Assignment3_extensions.ipynb` | **Extensions (for submission), offline only:** motor model, 3-state filter [−d/R, ωA, ωB] with the voltages as input, IMU filter, one comparison table incl. a missing encoder |
| `appendix_analysis.ipynb` | **Not for submission:** analysis details (calibration forms, synthetic simulation, k and σ_cal scans, electrical regression, per-channel NIS) |
| `INTERNAL-StateEstimation_StateFeedback/` | The Arduino sketch: template with `robot.cpp` and `state_estimator.cpp` completed. Compiles for the Arduino Mega (`arduino:avr:mega`) with the course libraries (now in `../libraries/`); not yet run on the robot |
| `../A2_prediction_model/Assignment2_solution.ipynb` | Assignment 2 prediction models (wheel odometry 1D, IMU 2D). Section 2.3 adds the coupling of heading uncertainty into the x, y covariance (EKF Jacobian of the rotation; the mean is unchanged), following the feedback on the submission. The 1D filter uses scenario 1 only, which is not affected. |

The notebook reads local recordings (Assignment 2 logs and QRoboticsCenter CSV files) that are **not** in this repo. Edit the paths and channel names in its "Settings" cell.

## The model

The template's `robot.h` defines `Meas2State = -1/R_WHEEL`, so the state is the **wheel angle relative to the wall**, `x = -d / R_WHEEL` [rad]. `R_WHEEL = 0.0325 m`, `TSAMPLE = 0.01 s`.

| Matrix | Value | Where it comes from |
|---|---|---|
| F | 1 | the wall does not move |
| B | `TSAMPLE`, applied to `v = (wA + wB)/2` [rad/s] | wheel odometry, the prediction model of Assignment 2 (scenario 1) |
| Q | `TSAMPLE^2 * (SIGMA_OMEGA^2 + (K_SLIP * v)^2)`, v = (wA + wB)/2: 3.3e-6 rad² standing still, 1.6e-5 rad² at 7 rad/s | `SIGMA_OMEGA = sigma_v / R_WHEEL` = 0.182 rad/s, `sigma_v` = 5.9 mm/s = spread of the wheel speed at constant voltage (Assignment 2, run R4). `K_SLIP` = 0.05: an empirical inflation of Q while driving (the 1D model is wrong when the robot curves or slips), tested on W1–W7 at 2.5 / 3.5 / 5 V: NIS while driving 1.16 / 1.00 / 1.46, i.e. ≈ 1 at 2.5 and 3.5 V and slightly over-confident at 5 V. k = 0.10 fits all speeds better (0.92 / 0.85 / 1.26); 0.05 is kept because it is the value in the firmware tested in C6, and the end error is 3.0 mm for every k. The odometry error is not proportional to speed (+6…7 % at 2.5 V, −1…−4 % at 3.5 V, +4…5 % at 5 V; curving explains ≤ 6 mm, the rest depends on the session), so it is not a slip model. Was 0.10 before the calibration refit |
| H | `-R_WHEEL` | the **calibrated** sensor measures `d = -R_WHEEL * x` |
| Calibration | `d = CAL_C2*m^2 + CAL_C1*m + CAL_C0`, m = `getFrontDistance()` | quadratic fit on the static recordings `data/A3/S_50mm` … `S_300mm` **and** the standstill points of W1–W3 (300 mm before, 42–45 mm after each drive; W4–W7 are validation only), one point per standstill window (tape distance from the robot's front edge; absorbs the library-curve bias of +17…+71 mm and the sensor mounting offset). At 30 cm the reading depends on the set-up (≈ 4 ADC counts between S and W), hence the drive-set-up points. Valid 42–300 mm; closed-loop targets 100–200 mm |
| R | `R(d) = (NOISE_C * dd/dm * (m - 0.0109)^2 / 24.7694)^2 + SIGMA_CAL^2`, evaluated every step | ADC quantisation: one count is worth more distance further away (≈ d²). NOISE_C = 0.83 counts fitted on the S recordings; σ ≈ 0.1 mm at 50 mm, 2.6 mm at 300 mm. SIGMA_CAL = 2 mm: floor for the calibration error (residual rms 1.2 mm, max ~3 mm), treated as white noise although it is systematic (pragmatic approximation); √R ≈ 2.0–3.2 mm |

The front sensor has drop-outs that read 24.78 m. `CorrectionUpdate` therefore has a **gate**:

* on the raw reading m: drop-outs, NaN and inf are skipped, and so is anything at or above 0.40 m (the calibration is fitted up to 0.371 m; beyond 30 cm the sensor over-reads, 40 cm reads 0.525 m, instead of dropping out; the quadratic folds back above 0.74 m);
* on the **calibrated** distance: readings outside 0.04–0.32 m are skipped (Sharp GP2Y0A41SK range ~0.04–0.30 m plus margin);
* innovations above 3 sigma (`nu^2 > 9 * S`) are skipped, **unless** 20 samples in a row (0.2 s) fail the test. Then the measurement is accepted again and `_Phat` is first widened to `P_RESYNC` (9.5 rad² ≈ 10 cm), so the filter actually catches up, because the estimate is wrong, not the sensor. The counter is cleared in `resetStateEstimator()`. Without this re-sync a wrong initial guess (e.g. ValueOut1 left at 0) locks the filter out forever. In the simulation (deliberately wrong start at 0 m, true 300 mm) the pure 3-sigma gate ends 312 mm off, the re-sync version 0.2 mm.

**Validation on the real drives W1–W7** (offline replay, main notebook section 6; start 300 mm; W1–W3 at 3.5 V, W4–W5 at 2.5 V, W6–W7 at 5 V; calibration without the replayed run):

| | W1 | W2 | W3 | W4 | W5 | W6 | W7 |
|---|---|---|---|---|---|---|---|
| filter end vs tape [mm] | +3.0 | +0.6 | +5.6 | +3.3 | +2.8 | +0.6 | +0.8 |
| NIS while driving (ideal 1) | 1.21 | 1.01 | 0.77 | 1.21 | 1.11 | 1.57 | 1.34 |
| 3σ rejections while driving / re-syncs | 10 / 0 | 0 / 0 | 1 / 0 | 1 / 0 | 3 / 1 | 10 / 0 | 6 / 0 |

End error vs tape: rms 3.0 mm, mean +2.4 mm. It is set by the calibration, not by the filter: after the stop the estimate converges to the calibrated sensor, the same for every Q and R. "While driving" = from the first wheel motion to 0.3 s after the voltage is switched off. Adding the W4–W7 standstill points to the calibration (appendix) changes it by up to −10 mm near 320 mm and ±3 mm in between: held-out end error rms 1.9 instead of 3.0 mm, but more rejections and re-syncs. Not adopted (firmware unchanged).

Simulation (appendix notebook, synthetic drives with a 2 mm calibration error): filter 1.6 mm RMSE, calibrated sensor 1.7 mm, prediction only 9.2 mm; starting at 0.45 m (outside the sensor range) 2.9 mm.

**Closed loop on the robot (C1–C5, main notebook section 8):** settles within 5 mm in 1.0–1.7 s, overshoot ≤ 1.5 mm in the plain steps (3 mm after the push in C5), holds the estimate on the target within 0.1 mm. The estimate ends 5–10 mm further than the tape because the calibrated sensor reads that much more in this set-up (standing still in it, `S_C150mm`: +8.9 mm vs tape, +10.9 mm vs ruler; `S_150mm`: +0.5 mm); the estimate equals the calibrated sensor within 0.1 mm. Subtracting the offset seen at the tape start lowers the end errors (rms 8.2 → 4.6 mm) but over-corrects (extensions notebook). Hand in front of the sensor (C4): rejected for 0.2 s, then re-sync. Hard start of C2: NIS 1.57 with k = 0.05, the same as the 5 V drives W6–W7 (1.57 / 1.34): k = 0.05 is slightly too small at high speed. C1–C5 were recorded with K = +2 and the flipped formula, identical to the template formula with K = −2.

## Values to fill in `robot.cpp`

```cpp
K(0) = -2.0;                           // [1/s] NEGATIVE with the template law (see below), tune it on the robot
_Phat(0,0) = P0_STATE;                 // (0.005 m / R_WHEEL)^2 = 0.0237 rad^2: ruler uncertainty of the start distance (state_estimator.h)
_xhat(0) = readValue(1)*Meas2State;    // initial distance in metres typed in ValueOut1 (checked: 0.04-0.30 m)
```

* Template law, unchanged: `desired_velocity = K * (_xhat - xref)`. Driving forward increases `x = -d/R_WHEEL`; too far from the wall means `xhat < xref`, so `xhat - xref < 0` and `K < 0` gives a positive (forward) wheel speed. Check the sign with the wheels lifted, then make `|K|` larger until the robot is fast but does not oscillate.
* Both inputs are in **metres**: `ValueOut0` = reference distance, `ValueOut1` = initial distance (ruler). The firmware converts them with `Meas2State` and ignores values outside 0.04–0.30 m (message in QRoboticsCenter).
* Do not change the velocity controller.

Log channels (`writeValue`, as in the template plus 11): 0 uA, 1 uB, 2 posA, 3 posB, 4 spA, 5 spB, 6 xref, 7 front distance (library curve, NOT calibrated) [m], 8 xhat (a priori), 9 xhat (a posteriori), 10 innovation (0 when the reading was rejected), 11 P (a posteriori) [rad^2]. Channels 9–11 are only written while the estimator is on.

## Differences from the course template

Reference: the GitLab template saved in [`../course_material/A3_1D_LKF/template/`](../course_material/A3_1D_LKF/template/) (fetched 2026-11-17; newer than the Sep 25 copy in the course zip). `robot.h` and the `.ino` are identical to it. `robot.cpp` differs only where the template has `??` (`K`, `_Phat`, `_xhat`) and in the rows below; comment-only changes are not listed. `state_estimator.cpp/.h` are ours (the template has only `??`; `ResetGate()`, `P0_STATE`, `P_RESYNC` are added there).

| Change | Why |
|---|---|
| `xref(0) = readValue(0)*Meas2State` (in `init()` and in the controller; template: `readValue(0)`) | Unit conversion of the operator input only, so ValueOut0 and ValueOut1 are both in metres; the control law itself is unchanged. |
| Input check: ValueOut0 / ValueOut1 outside 0.04–0.30 m are ignored, with a message (reference: the robot holds; start: start mid-range with `P_RESYNC`, so the estimator syncs to the sensor) | Catches cm/m typos and missing values, which would otherwise drive the robot into the wall or start the filter far off. |
| `ResetGate()` in `resetStateEstimator()` | The gate's rejected-reading counter lives in `state_estimator.cpp`. Without clearing it, a new run inherits the rejections of the previous one. |
| `writeValue(11, _Phat(0,0))` | Logs the a-posteriori variance, to check the filter's uncertainty on the robot (channel 11 is unused in the template). |

## Tests to run on the robot

1. **Sensor noise.** Robot still about 20 cm from the wall, no buttons, record 15 s. Save as `S_<dist>mm.csv` (e.g. `S_200mm.csv`). Done for 50–300 mm: the main notebook (sections 2–3) fits the calibration and R(d) on these files.
2. **Estimator only.** Start about 30 cm from the wall (inside the sensor range, 4–30 cm), measure the start distance with the ruler and type it in ValueOut1, press Button 1, push the robot by hand to about 20 cm, record as `W<n>_start<mm>_end<mm>.csv` (start and end distance from the ruler), measure the end distance with a ruler. Replay offline in section 5 of the notebook and compare with channel 9.
3. **Closed loop.** Wheels lifted: check the direction. Floor: type the ruler distance in ValueOut1 and the reference in ValueOut0, press Button 1 then Button 0, tune `K`, record the best run.
4. **Robustness (optional).** Wave a hand in front of the sensor to show the gate works; start with a wrong initial distance to show convergence.

## Instructions for a generative AI assistant (Claude, ChatGPT, ...)

See [`../CLAUDE.md`](../CLAUDE.md).
