# Team rules for Claude Code (read before every task)

Two people work on this repo remotely, each with their own Claude Code session.
**Toghrul** has the robot (flashing, recordings, on-robot tests). **David** works without the robot (notebooks, code review, firmware edits).

## Before and after every task
- `git pull` before changing anything. If the pull brings changes to files you're about to edit, re-read them first.
- Commit small, push right after. One topic per commit, message says what and why.
- Never push firmware that doesn't compile: `arduino-cli compile --fqbn arduino:avr:mega --libraries libraries <sketch>` (Arduino IDE ships arduino-cli).
- Don't edit a notebook the other person is working on (ask in chat first). Notebooks merge badly.
- Don't "fix" a convention below without the user explicitly asking. If something looks wrong, report it instead.
- Read the code before you answer, don't go from memory: the state unit (`Meas2State`), the logged channels (`writeValue`) and the assignment notebook in `course_material/`.

## Conventions (agreed, don't change silently)
- **State (A3):** x = -d / R_WHEEL [rad] (`Meas2State = -1/R_WHEEL`), d = distance to the wall [m]. x grows when driving forward.
- **Controller:** template law `desired_velocity = K * (xref - _xhat)`, K > 0. Only K may differ from the course template in the position controller. The velocity controller is never changed.
- **QRC inputs:** ValueOut0 = reference distance [m], ValueOut1 = start distance [m] (both converted with Meas2State).
- **Sensor:** `getFrontDistance()` returns metres from the library curve, which over-reads this sensor (+17 mm at 5 cm to +71 mm at 30 cm). `CorrectionUpdate` calibrates every reading first (quadratic in m, fitted on `data/A3/S_*` and the standstill points of W1–W3, one point per standstill window; d = tape distance from the robot's front edge, valid 42–300 mm). At 30 cm the reading depends on the set-up, so calibration points are recorded in the drive set-up; closed-loop targets 100–200 mm. Sharp GP2Y0A41SK, valid ~0.04–0.30 m. Drop-outs read ~24.78 m. Gate: raw m must be < 0.40 m (drop-outs, NaN/inf; calibration fitted up to 0.371 m; beyond 30 cm the sensor over-reads instead of dropping out); calibrated d must be in 0.04–0.32 m. Below 4 cm the sensor reads too far – keep the robot > 4 cm from the wall.
- **Filter:** prediction model = A2 scenario 1 (wheel odometry): F = 1, B = TSAMPLE, u = mean wheel speed (wA + wB)/2 [rad/s]. H = -R_WHEEL, Q(ω) = TSAMPLE² (SIGMA_OMEGA² + (K_SLIP ω)²) per step: SIGMA_OMEGA from A2 (Q = 3.30e-6 rad² standing still), K_SLIP = 0.05 chosen on W1–W3 (mean NIS 0.93 while driving with the final calibration; was 0.10 before the calibration refit; speed dependence not tested yet, all W runs at 3.5 V), R(d) in m², evaluated every step: ADC quantisation model with NOISE_C counts fitted on the S recordings, plus a calibration floor SIGMA_CAL² (2 mm). P0_STATE = (5 mm / R_WHEEL)² (the start distance is tape-measured and typed in ValueOut1) and P_RESYNC = 9.5 rad² (≈ 10 cm, used by the gate) are defined once in state_estimator.h.
- **Notebook and firmware must match:** same F, B, Q, H, calibration, R(d), P0, P_RESYNC, gate. If one changes, change the other in the same commit.
  - The **firmware is the reference**. The notebook does not generate C++. It has a consistency-check cell that reads the values from `state_estimator.h/.cpp` (SIGMA_OMEGA, K_SLIP, H, P0, P_RESYNC, calibration CAL_C0..2, SHARP_A/B, NOISE_C, SIGMA_CAL, gate limits M_RAW_MAX/D_MIN/D_MAX, gate threshold, re-sync count) and asserts they equal its own. That cell must pass before pushing.
  - The Python simulation implements the same gate as `CorrectionUpdate`: every reading is calibrated, raw m ≥ 0.40 m (drop-outs, NaN/inf) and calibrated d outside 0.04–0.32 m are skipped, R(d) is used in the update and the innovation test; `nu² > 9 S` is skipped unless it happens more than 20 times in a row, and then P is widened to P_RESYNC before the update; the counter starts at 0 for every run (`ResetGate()`).
  - The simulation starts like the robot: inside the sensor range (0.30 m), from the tape-measured start distance with P0. A wrong start estimate is only used to demonstrate the re-sync, and is labelled as such.
- Constants, column maps and A2 results live in `mr_common.py` – import them, don't redefine.

## Data
- All recordings go to `data/A2/` or `data/A3/`, with one row per file in `LOGBOOK.md` (date, test, setup, measured distances, notes).
- File names: `S_<dist>mm.csv` (static sensor), `W<n>_start<mm>_end<mm>.csv` (drive to wall), `M<n>_lifted.csv` (voltage steps).
- Never edit or delete a CSV; add a new one instead.
- Noise values come from real data, never invented: sigma_v from the steady parts of a constant-voltage drive, sensor calibration and R(d) from static recordings at tape-measured distances (S files). Look at the raw CSV to check which column is the front distance (stable when still, drop-outs at 24.78 m). If a channel is inferred from the data and not from the source code, say so.

## Robot tests
- Guide the user through the tests in `A3_1D_LKF/README.md` one at a time. Ask for the recording before claiming anything works.
- Replay every real recording offline through the same filter as the notebook and compare with the on-board estimate (channel 9, a posteriori).

## Sketches
- `test_firmware/INTERNAL-Identification-A3/` – A3 recordings (button 1 wall drive, button 2 voltage steps lifted).
- `A3_1D_LKF/INTERNAL-StateEstimation_StateFeedback/` – closed-loop filter + controller (the graded firmware).
- Folder name must equal the .ino name (Arduino IDE requirement).

## Style
- Simple, readable code; no clever abstractions. Both users are mechatronics students, not software engineers.
- Notebooks: condensed explanations, course notation (x, P, F, B, u, Q, H, R), run top to bottom before pushing.
- The A3 notebook covers: model, noise estimation, a simulated wall approach (sensor noise from data, drop-outs, a few % unknown wheel-speed scale error, wrong initial guess), filter vs. sensor vs. prediction only, the effect of the gate, a consistency check (normalised innovation squared, mean ≈ 1), and a sensitivity check on scaling Q and R. Say clearly that the wall approach is simulated until a real recording exists.
- Be honest about limits: simulated results are not robot results; R measured at one distance only (sensor noise grows with distance); the wheel-speed error is partly a bias that white-noise Q does not represent (a larger Q may help, check with the innovation test).
- This assignment is defended orally: after each change, give a short summary of the reasoning the user must be able to explain.
