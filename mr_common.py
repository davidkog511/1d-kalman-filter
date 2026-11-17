"""Shared code for the Mobile Robotics assignments (KU Leuven, MECOtron robot).

Import from a notebook one folder down (A2_prediction_model/, A3_1D_LKF/, ...):

    import sys; sys.path.insert(0, '..')
    from mr_common import *

Contents
  1. Robot constants                    (from libraries/control-theory/mecotron.h)
  2. Paths                              (DATA_A2, DATA_A3, relative to this file)
  3. load_log()                         (QRoboticsCenter CSV -> DataFrame, column map per test firmware)
  4. Assignment 2 results               (constants + the functions that produce them)

Rule: change a value here only together with the notebook it comes from, and re-run that notebook.
"""
from pathlib import Path

import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------------------------
# 1. Robot constants (libraries/control-theory/mecotron.h)
# ---------------------------------------------------------------------------------------------
R_WHEEL     = 0.0325   # wheel radius [m]                                   mecotron.h: R_WHEEL
WHEELBASE   = 0.166    # distance between the wheels [m]                    mecotron.h
ENCODER_CPR = 1792     # encoder counts per wheel revolution                mecotron.h: ENC_TO_RAD = pi/28/16/2
DT          = 0.01     # sample time [s]                                    mecotron.h: TSAMPLE
G           = 9.81     # gravity [m/s^2]

# ---------------------------------------------------------------------------------------------
# 2. Paths
# ---------------------------------------------------------------------------------------------
REPO    = Path(__file__).resolve().parent
DATA_A2 = REPO / 'data' / 'A2'
DATA_A3 = REPO / 'data' / 'A3'

# ---------------------------------------------------------------------------------------------
# 3. Logs: column maps per test firmware (ValueIn0 ... ValueIn11 of QRoboticsCenter, in order)
# ---------------------------------------------------------------------------------------------
COLUMN_MAPS = {
    # test_firmware/INTERNAL-Identification (our version, R1-R5 of Assignment 2)
    'identification': ['VA', 'VB', 'wA', 'wB', 'pA', 'pB', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'],
    # course default of INTERNAL-Identification (assignments-main, before our changes)
    'identification_course': ['VA', 'VB', 'IA', 'IB', 'pA', 'pB', 'wA', 'wB', 'front', 'side', 'ax', 'gx'],
    # A3_1D_LKF/INTERNAL-StateEstimation_StateFeedback (writeValue 0..10). xhat_post and nu are only
    # written while the estimator is enabled (Button 1). x* are in rad (x = -d / R_WHEEL), front in m.
    'state_feedback': ['uA', 'uB', 'pA', 'pB', 'wA', 'wB', 'xref', 'front', 'xhat_prior', 'xhat_post', 'nu'],
}
# Units: V [V], I [A], w [rad/s of the wheel], p [rad of the wheel], a [m/s^2], g [rad/s], distances [m].


def load_log(filename, firmware='identification'):
    """Read a QRoboticsCenter log. Adds t = time since the start of the log [s].

    filename : path to the CSV (first line is the QRC header, second line the column names)
    firmware : key of COLUMN_MAPS, i.e. which robot.cpp wrote the log (see LOGBOOK.md)
    """
    columns = COLUMN_MAPS[firmware]
    df = pd.read_csv(filename, header=1)
    if len(df.columns) - 1 < len(columns):
        raise ValueError(f'{filename}: {len(df.columns) - 1} value columns, firmware {firmware!r} expects {len(columns)}')
    if len(df.columns) - 1 > len(columns):
        df = df.iloc[:, :len(columns) + 1].copy()   # QRC always writes 12 channels; drop the unused ones
    df.columns = ['Time'] + columns
    df['t'] = (df['Time'] - df['Time'].iloc[0]) / 1000.0
    return df


def window(df, t_start, t_end):
    """Rows with t_start <= t < t_end."""
    return df[(df.t >= t_start) & (df.t < t_end)]


# ---------------------------------------------------------------------------------------------
# 4. Assignment 2 results (A2_prediction_model/Assignment2_solution.ipynb, logs data/A2/R1-R5)
#    The constants are the values the notebook computes; the functions below are the same code,
#    so the constants can be re-derived with a2_results().
# ---------------------------------------------------------------------------------------------

# Scenario 1, wheel odometry (notebook cell "pooled_std"): spread of the wheel speed at constant 6 V in R4
STEADY_R4 = [(31.9, 32.45), (33.0, 33.6)]   # [s] steady, straight parts of the 6 V drive in R4 (slip at 32.5-32.9 s excluded)
SIGMA_WA  = 0.2366862709492524              # [rad/s] wheel A
SIGMA_WB  = 0.27560884286923865             # [rad/s] wheel B
SIGMA_V   = 0.005903484887902763            # [m/s]   forward speed, R_WHEEL*sqrt(sA^2+sB^2)/2
Q1        = 3.48511338216963e-09            # [m^2 per step] (DT*SIGMA_V)^2. In rad (A3 state): Q1 / R_WHEEL^2

# Scenario 2, IMU calibration (notebook section 2.1)
# Bias method: subtract the mean of the run's OWN static window (biases drift between runs), see imu_bias().
STATIC_R4  = (5, 29.5)                      # [s] static window used for R4
IMU_BIAS_R1 = {'ax': 0.05499839277839713, 'ay': -0.054272391763904373, 'az': -0.9922701102440232,   # [m/s^2], az minus G
               'gx': -0.04201782426607209, 'gy': -0.029691767694785084, 'gz': 0.05225727116313638}  # [rad/s]
PSI        = -3.1261723587308228            # [rad] robot forward direction in IMU axes (~181 deg: IMU x backward, y right)
SIGMA_PSI  = 0.07465196327044148            # [rad] spread of the three PSI estimates (~4.3 deg), section 2.3
R_ARM      = np.array([-0.07771248332964019, 0.05782581123062076])   # [m] IMU position in robot axes: 78 mm behind, 58 mm left of the axle middle


def pooled_std(df, column, windows):
    """Standard deviation around the mean of each window, pooled over the windows (notebook cell [4])."""
    deviations = [window(df, a, b)[column] - window(df, a, b)[column].mean() for a, b in windows]
    return np.concatenate(deviations).std()


def wheel_speed_noise(R4, windows=STEADY_R4):
    """sigma_wA, sigma_wB [rad/s], sigma_v [m/s] and Q1 [m^2] from the constant-voltage drive."""
    sigma_wA = pooled_std(R4, 'wA', windows)
    sigma_wB = pooled_std(R4, 'wB', windows)
    sigma_v = R_WHEEL * np.sqrt(sigma_wA**2 + sigma_wB**2) / 2
    return sigma_wA, sigma_wB, sigma_v, (DT * sigma_v)**2


def imu_bias(df, t_static, columns=('ax', 'ay', 'gx', 'gy', 'gz')):
    """Bias = mean over the run's own static window (az is left out: it contains gravity)."""
    return window(df, *t_static)[list(columns)].mean()


def imu_to_robot(vx, vy, psi=PSI):
    """Rotate a vector from IMU axes to robot axes (psi = robot forward direction in IMU axes)."""
    return vx * np.cos(psi) + vy * np.sin(psi), -vx * np.sin(psi) + vy * np.cos(psi)


def mounting_yaw(R1, R3, R4):
    """PSI and SIGMA_PSI from the lifted side (R3) and the start/stop impulses of R4 (notebook cells [10], [19])."""
    angle = lambda dx, dy: np.degrees(np.arctan2(dy, dx))
    acc_R4 = window(R4, *STATIC_R4)[['ax', 'ay']].mean()
    lift = R3[['ax', 'ay']].mean() - R1[['ax', 'ay']].mean()
    start = (window(R4, 31.6, 31.9)[['ax', 'ay']] - acc_R4).sum()
    stop = (window(R4, 33.6, 33.9)[['ax', 'ay']] - acc_R4).sum()
    ang = np.radians([angle(lift.ax, lift.ay) - 90, angle(start.ax, start.ay), angle(-stop.ax, -stop.ay)])
    psi = np.arctan2(np.mean(np.sin(ang)), np.mean(np.cos(ang)))
    sigma_psi = np.std(np.angle(np.exp(1j * (ang - psi))), ddof=1)
    return psi, sigma_psi


def lever_arm(R5, psi=PSI, spins=((7.5, 12.5), (15.5, 21.5))):
    """IMU position r [m] in robot axes from the centripetal acceleration a = -w^2 r in R5 (notebook cell [10])."""
    rest = window(R5, 0, 5)
    r_list = []
    for a, b in spins:
        seg = window(R5, a, b)
        wz = (seg.gz - rest.gz.mean()).mean()
        r_imu = -(seg[['ax', 'ay']].mean() - rest[['ax', 'ay']].mean()).values / wz**2
        r_list.append(imu_to_robot(r_imu[0], r_imu[1], psi))
    return np.mean(r_list, axis=0)


def a2_results(data=DATA_A2):
    """Recompute all Assignment 2 constants above from the R1-R5 logs (for checking)."""
    R1, R3, R4, R5 = (load_log(data / f) for f in ['R1_static.csv', 'R3_left_up.csv',
                                                  'R4_straight_fwd700mm_right190mm_turnR15deg.csv', 'R5_rotation_CW.csv'])
    sigma_wA, sigma_wB, sigma_v, q1 = wheel_speed_noise(R4)
    psi, sigma_psi = mounting_yaw(R1, R3, R4)
    bias = R1[['ax', 'ay', 'az', 'gx', 'gy', 'gz']].mean() - np.array([0, 0, G, 0, 0, 0])
    return dict(SIGMA_WA=sigma_wA, SIGMA_WB=sigma_wB, SIGMA_V=sigma_v, Q1=q1, PSI=psi, SIGMA_PSI=sigma_psi,
                R_ARM=lever_arm(R5, psi), IMU_BIAS_R1=bias.to_dict())


if __name__ == '__main__':
    # python mr_common.py  ->  checks that the constants still match the data
    res = a2_results()
    for name, value in res.items():
        stored = globals()[name]
        if isinstance(value, dict):
            ok = all(np.isclose(value[k], stored[k], rtol=1e-12, atol=0) for k in stored)
        else:
            ok = np.allclose(value, stored, rtol=1e-12, atol=0)
        print(f'{name:12s} {"OK" if ok else "MISMATCH"}  {value}')
