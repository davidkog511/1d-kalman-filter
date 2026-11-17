#include "state_estimator.h"

// 1D linear Kalman filter for the distance to the wall in front of the robot.
//   state   x = wheel angle relative to the wall [rad] = -d / R_WHEEL   (Meas2State in robot.h)
//   input   u = [speed motor A, speed motor B] [rad/s]  ->  v = (uA + uB) / 2   (Assignment 2, scenario 1)
//   measure y = front distance [m],  H = -R_WHEEL
//   sigma_v = 5.90 mm/s = 0.182 rad/s (wheel speed at constant voltage) -> Q = (TSAMPLE*sigma_omega)^2
//   sigma_d = 21.8 mm   (sensor, robot still)                            -> R = sigma_d^2

static const float Q_PROCESS = 3.299516e-06f;   // [rad^2]
static const float R_MEAS    = 4.751152e-04f;   // [m^2]
static const float GATE      = 9.0f;            // innovation gate nu^2 / S (3 sigma): rejects implausible measurements
static const float D_MIN     = 0.04f;           // [m] valid range of the Sharp GP2Y0A41SK: ~0.04-0.30 m;
static const float D_MAX     = 0.35f;           // [m] 0.35 m upper limit for margin (drop-outs read 24.78 m)
static const int   MAX_REJECT = 20;             // after 20 gated samples in a row (0.2 s), accept again: recovers from a wrong initial guess
static int rejected = 0;                        // number of consecutive gated samples

void ResetGate() {
  rejected = 0;
}


void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat,BLA::Matrix<1> &_Phat ) {

  // System F&B-matrix
  BLA::Matrix<1,1> F = { 1.0f };
  BLA::Matrix<1,1> B = { TSAMPLE };

  BLA::Matrix<1,1>Q = { Q_PROCESS };
  BLA::Matrix<1,1>v = { 0.5f * (u(0) + u(1)) };   // mean wheel speed [rad/s]

  // Evaluate discrete-time system dynamics
  xhat = F*xhat + B*v;
  _Phat = F*_Phat* ~F + Q;
}

void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat) {
  // Tuning parameter
  BLA::Matrix<1,1> R = { R_MEAS };

  // Compute Jacobian of measurement equation
  BLA::Matrix<1,1> H = { -R_WHEEL };              // distance = -R_WHEEL * x

  // Compute innovation
  nu = y - H*xhat;

  // Set estimator gain
  BLA::Matrix<1,1> S = H * _Phat * ~H + R;

  // Reject readings outside the sensor range (also catches inf/NaN), and implausible measurements
  // unless they persist (then the estimate is wrong, not the sensor). nu = 0 marks a rejected reading.
  if (!(y(0) >= D_MIN && y(0) <= D_MAX)) {
    nu(0) = 0.0f;
    return;
  }
  if (nu(0)*nu(0) > GATE*S(0,0)) {
    if (++rejected <= MAX_REJECT) {
      nu(0) = 0.0f;
      return;
    }
    // Re-sync: the estimate is off. Widen P so this measurement pulls the estimate back.
    _Phat(0,0) = P0_STATE;
    S = H * _Phat * ~H + R;
  }
  rejected = 0;

  BLA::Matrix<1,1> K = _Phat * ~H * Inverse(S);

  // Compute corrected system state estimate
  xhat += K * nu;
  BLA::Matrix<1,1> I = { 1.0f };
  _Phat = ( I- K*H)*_Phat;
}
