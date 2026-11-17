# Mobile Robotics (KU Leuven): team repository

This repo holds our assignment work for the MECOtron robot: notebooks, Arduino firmware, recorded logs and the course material they build on. Each assignment builds on the earlier ones (the A3 Kalman filter reuses the A2 prediction model), so shared constants and results live in one place, `mr_common.py`.

Team: David Cogîlniceanu, Toghrul Alibayov.

## Folder map

```
README.md              this file
LOGBOOK.md             one row per recorded log: date, test, setup, notes
mr_common.py           robot constants, load_log() with column maps per firmware, A2 results
.gitignore

data/A2/               R1-R5 logs (QRoboticsCenter CSV) used by Assignment 2
data/A3/               logs for Assignment 3 (sensor_still.csv, wall_approach.csv, ...)

A2_prediction_model/   Assignment2_solution.ipynb  (wheel odometry 1D, IMU 2D, heading-position covariance in 2.3)
A3_1D_LKF/             Assignment3_LKF.ipynb, README.md (model, values, robot tests)
  INTERNAL-StateEstimation_StateFeedback/   Arduino sketch for A3 (folder name = .ino name)
test_firmware/
  INTERNAL-Identification/                  Arduino sketch that recorded R1-R5 (buttons 1/2 = test sequences)
  INTERNAL-Identification-A3/               A3 recordings: button 1 = drive to wall, button 2 = voltage steps (LIFTED)
libraries/             Arduino libraries from the course (MECOtron, BasicLinearAlgebra, Adafruit, ...)

course_material/       READ-ONLY copies of the course notebooks, images, datasheet and FAQ
```

Don't edit `course_material/`. Copy a notebook out of it if you want to work on it.

## Running the notebooks

1. Python 3.10+ with `numpy pandas matplotlib scipy jupyter`. The A3 notebook also needs `scikit-learn`, `statsmodels` and `plotly`, but only for the course appendix cell.
2. Open the notebook from **its own folder** (e.g. `jupyter lab` in the repo root, then open `A2_prediction_model/Assignment2_solution.ipynb`). The notebooks do `sys.path.insert(0, '..')` to import `mr_common`, and read logs from `data/` through `mr_common.DATA_A2` / `DATA_A3`.
3. To read a new log: `load_log(DATA_A3 / 'file.csv', firmware='state_feedback')`. The `firmware` key says which `robot.cpp` wrote the log, so the columns get the right names (see `COLUMN_MAPS` in `mr_common.py` and the firmware column in `LOGBOOK.md`).
4. Check that the stored A2 constants still match the data with `python mr_common.py`. Every line should say `OK`.

## Flashing the Arduino

The robot uses an Arduino Mega (`arduino:avr:mega`). The libraries are in this repo, so you don't need to install anything else.

**Arduino IDE:** set *File → Preferences → Sketchbook location* to the **repo root** (the folder that contains `libraries/`), then restart the IDE. Open the `.ino` file (e.g. `A3_1D_LKF/INTERNAL-StateEstimation_StateFeedback/INTERNAL-StateEstimation_StateFeedback.ino`), select *Arduino Mega or Mega 2560* and the COM port, and upload.

**arduino-cli:**
```
arduino-cli compile --fqbn arduino:avr:mega --libraries libraries A3_1D_LKF/INTERNAL-StateEstimation_StateFeedback
arduino-cli upload  --fqbn arduino:avr:mega -p COM3 A3_1D_LKF/INTERNAL-StateEstimation_StateFeedback
```

Then open QRoboticsCenter, connect, and use the buttons and ValueOut channels described in the sketch's README or the assignment README. Build output (`build/`, `*.hex`, `*.elf`) is ignored by git.

## Team workflow

* **Roles:** Toghrul records on the robot and pushes the logs. David writes the code (notebooks, `mr_common.py`).
* **Pull before you start, every time:** `git pull`. Push when you stop.
* **Never edit the same notebook at the same time.** Notebook JSON doesn't merge well. Say in the chat which notebook you're opening, and commit and push before handing over.
* **New recording:**
  1. Put the CSV in `data/A<n>/` with a descriptive name (what, distance, voltage), not `log_gpio_<date>.csv`.
  2. Add a row to `LOGBOOK.md` (date, test, setup, notes, which firmware).
  3. Commit the CSV and the LOGBOOK row together.
* **Never change a CSV after committing it.** If a test went wrong, record a new one and add a note in the LOGBOOK.
* **Shared values** (R_WHEEL, DT, sigma_v, Q, ...) come from `mr_common.py`. Don't copy numbers into notebooks.
