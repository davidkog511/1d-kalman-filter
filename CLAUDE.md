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
- **Sensor:** `getFrontDistance()` returns metres (Sharp GP2Y0A41SK, valid ~0.04–0.30 m). Drop-outs read ~24.78 m. Gate uses 0.04–0.35 m. Below 4 cm the sensor reads too far – keep the robot > 4 cm from the wall.
- **Filter:** prediction model = A2 scenario 1 (wheel odometry): F = 1, B = TSAMPLE, u = mean wheel speed (wA + wB)/2 [rad/s]. H = -R_WHEEL, Q = 3.30e-6 rad² per step (= A2 Q1 / R_WHEEL²), R in m² from data/A3 static recordings, P0 defined once in state_estimator.h.
- **Notebook and firmware must match:** same F, B, Q, H, R, P0, gate. If one changes, change the other in the same commit.
  - The **firmware is the reference**. The notebook does not generate C++. It has a consistency-check cell that reads the values from `state_estimator.h/.cpp` (Q, R, H, P0, gate limits D_MIN/D_MAX, gate threshold, re-sync count) and asserts they equal its own. That cell must pass before pushing.
  - The Python simulation implements the same gate as `CorrectionUpdate`: readings outside 0.04–0.35 m are skipped; `nu² > 9 S` is skipped unless it happens more than 20 times in a row, and then P is reset to P0 before the update; the counter starts at 0 for every run (`ResetGate()`).
- Constants, column maps and A2 results live in `mr_common.py` – import them, don't redefine.

## Data
- All recordings go to `data/A2/` or `data/A3/`, with one row per file in `LOGBOOK.md` (date, test, setup, measured distances, notes).
- File names: `S_<dist>mm.csv` (static sensor), `W<n>_start<mm>_end<mm>.csv` (drive to wall), `M<n>_lifted.csv` (voltage steps).
- Never edit or delete a CSV; add a new one instead.
- Noise values come from real data, never invented: sigma_v from the steady parts of a constant-voltage drive, sigma_d from a part of a recording where the robot stands still. Look at the raw CSV to check which column is the front distance (stable when still, drop-outs at 24.78 m). If a channel is inferred from the data and not from the source code, say so.

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
