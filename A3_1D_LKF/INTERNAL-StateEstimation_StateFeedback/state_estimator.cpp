#include "state_estimator.h"

// 1D linear Kalman filter for the distance to the wall in front of the robot.
//   state   x = wheel angle relative to the wall [rad] = -d / R_WHEEL   (Meas2State in robot.h)
//   input   u = [speed motor A, speed motor B] [rad/s]  ->  v = (uA + uB) / 2   (Assignment 2, scenario 1)
//   measure y = getFrontDistance() [m] (library curve), calibrated to d [m] before the update,  H = -R_WHEEL
//   Q(v) = TSAMPLE^2 * (SIGMA_OMEGA^2 + (K_SLIP*v)^2): wheel-speed noise (A2, at constant voltage) plus a
//   speed-dependent part for slip (odometry error grows with the distance driven, sign changes between runs)
//   R(d) from the ADC quantisation of the sensor, fitted on data/A3/S_50mm ... S_300mm
// All numbers come from A3_1D_LKF/Assignment3_LKF.ipynb (section 2.2), which checks that they match.

static const float SIGMA_OMEGA = 1.8164569e-01f; // [rad/s] = sigma_v / R_WHEEL, sigma_v = 5.90 mm/s (A2, run R4)
static const float K_SLIP      = 0.05f;          // [-] slip noise per unit wheel speed, chosen on W1-W3 with the final
                                                 //     calibration (mean NIS 0.93 while driving); was 0.10 with the S-only
                                                 //     calibration, when part of the mismatch was calibration error, not slip

// Calibration: d = CAL_C2*m^2 + CAL_C1*m + CAL_C0, m = getFrontDistance() [m], d = tape distance from the
// robot's front edge [m] (absorbs the library-curve error and the sensor mounting offset). Fitted on the static
// recordings S_50 ... S_300 and the standstill points of the drives W1-W3 (300 mm and 42-45 mm, drive set-up),
// one point per standstill window. Valid 42-300 mm; closed-loop targets 100-200 mm.
static const float CAL_C2 = -8.1112216e-01f;   // [1/m]
static const float CAL_C1 =  1.2277406e+00f;   // [-]
static const float CAL_C0 = -3.5707847e-02f;   // [m]

// Library curve of the sensor (sharp41S.cpp): m = SHARP_A/raw + SHARP_B, raw = ADC counts
static const float SHARP_A = 24.7694f;          // [m * count]
static const float SHARP_B = 0.0109311f;        // [m]
// Measurement noise: NOISE_C ADC counts (std), so sigma_d = NOISE_C * dd/dm * (m - SHARP_B)^2 / SHARP_A
static const float NOISE_C = 8.2967843e-01f;   // [counts]
// Calibration floor: the calibration is only as good as the tape placement (residual rms 1.2 mm, max ~3 mm).
// Added to R as if it were white noise: a pragmatic approximation, the calibration error is systematic.
static const float SIGMA_CAL = 0.002f;          // [m]

static const float M_RAW_MAX = 0.40f;           // [m] limit on m: drop-outs read 24.78, NaN/inf. The calibration is fitted up to
                                                //     m = 0.371 m, beyond 30 cm the sensor over-reads (40 cm reads 0.525 m),
                                                //     and the quadratic folds back above m = 0.74 m
static const float D_MIN     = 0.04f;           // [m] valid range of the CALIBRATED distance (Sharp GP2Y0A41SK: ~0.04-0.30 m)
static const float D_MAX     = 0.32f;           // [m] 0.30 m plus margin
static const float GATE      = 9.0f;            // innovation gate nu^2 / S (3 sigma): rejects implausible measurements
static const int   MAX_REJECT = 20;             // after 20 gated samples in a row (0.2 s), accept again: recovers from a wrong initial guess
static int rejected = 0;                        // number of consecutive gated samples

void ResetGate() {
  rejected = 0;
}


void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat,BLA::Matrix<1> &_Phat ) {

  // System F&B-matrix
  BLA::Matrix<1,1> F = { 1.0f };
  BLA::Matrix<1,1> B = { TSAMPLE };

  BLA::Matrix<1,1>v = { 0.5f * (u(0) + u(1)) };   // mean wheel speed [rad/s]
  float q_w = SIGMA_OMEGA*SIGMA_OMEGA + K_SLIP*K_SLIP*v(0)*v(0);   // variance of the speed error [rad^2/s^2]
  BLA::Matrix<1,1>Q = { TSAMPLE*TSAMPLE*q_w };     // Q = B q_w B^T [rad^2]

  // Evaluate discrete-time system dynamics
  xhat = F*xhat + B*v;
  _Phat = F*_Phat* ~F + Q;
}

void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat) {
  float m = y(0);                                 // library distance [m]

  // Reject drop-outs (24.78 m), NaN and inf on the raw value. nu = 0 marks a rejected reading.
  if (!(m > SHARP_B && m < M_RAW_MAX)) {
    nu(0) = 0.0f;
    return;
  }

  // Calibrate, then check the range on the calibrated distance
  float d = (CAL_C2*m + CAL_C1)*m + CAL_C0;       // [m]
  if (!(d >= D_MIN && d <= D_MAX)) {
    nu(0) = 0.0f;
    return;
  }

  // Measurement noise R(d): one ADC count is worth dd/draw = dd/dm * (m - SHARP_B)^2 / SHARP_A metres,
  // plus the calibration floor SIGMA_CAL^2
  float dd_dm = 2.0f*CAL_C2*m + CAL_C1;
  float sigma_d = NOISE_C * dd_dm * (m - SHARP_B)*(m - SHARP_B) / SHARP_A;
  BLA::Matrix<1,1> R = { sigma_d*sigma_d + SIGMA_CAL*SIGMA_CAL };

  // Compute Jacobian of measurement equation
  BLA::Matrix<1,1> H = { -R_WHEEL };              // distance = -R_WHEEL * x

  // Compute innovation
  BLA::Matrix<1> z = { d };
  nu = z - H*xhat;

  // Set estimator gain
  BLA::Matrix<1,1> S = H * _Phat * ~H + R;

  // Reject implausible measurements unless they persist (then the estimate is wrong, not the sensor)
  if (nu(0)*nu(0) > GATE*S(0,0)) {
    if (++rejected <= MAX_REJECT) {
      nu(0) = 0.0f;
      return;
    }
    // Re-sync: the estimate is off. Widen P so this measurement pulls the estimate back.
    _Phat(0,0) = P_RESYNC;
    S = H * _Phat * ~H + R;
  }
  rejected = 0;

  BLA::Matrix<1,1> K = _Phat * ~H * Inverse(S);

  // Compute corrected system state estimate
  xhat += K * nu;
  BLA::Matrix<1,1> I = { 1.0f };
  _Phat = ( I- K*H)*_Phat;
}
