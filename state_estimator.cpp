#include "state_estimator.h"

// 1D linear Kalman filter for the distance to the wall in front of the robot.
//   state   x = wheel angle relative to the wall [rad] = -d / R_WHEEL   (Meas2State in robot.h)
//   input   u = [speed motor A, speed motor B] [rad/s]  ->  v = (uA + uB) / 2   (Assignment 2, scenario 1)
//   measure y = front distance [m],  H = -R_WHEEL
//   sigma_v = 5.90 mm/s = 0.182 rad/s (wheel speed at constant voltage) -> Q = (TSAMPLE*sigma_omega)^2
//   sigma_d = 21.8 mm   (sensor, robot still)             -> R = sigma_d^2

static const float Q_PROCESS = 3.299516e-06f;   // [rad^2]
static const float R_MEAS    = 4.751152e-04f;   // [m^2]
static const float GATE      = 9.0f;       // innovation gate nu^2 / S (3 sigma): rejects sensor drop-outs

void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat, BLA::Matrix<1> &_Phat) {
  BLA::Matrix<1,1> F = { 1.0f };
  BLA::Matrix<1,1> B = { TSAMPLE };
  BLA::Matrix<1,1> Q = { Q_PROCESS };
  BLA::Matrix<1,1> v = { 0.5f * (u(0) + u(1)) };           // mean wheel speed [rad/s]

  xhat = F*xhat + B*v;
  _Phat = F*_Phat*~F + Q;
}

void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu, BLA::Matrix<1> &_Phat) {
  BLA::Matrix<1,1> R = { R_MEAS };
  BLA::Matrix<1,1> H = { -R_WHEEL };                        // distance = -R_WHEEL * x

  nu = y - H*xhat;
  BLA::Matrix<1,1> S = H*_Phat*~H + R;

  if (nu(0)*nu(0) > GATE*S(0,0)) return;                    // implausible measurement (drop-out)

  BLA::Matrix<1,1> K = _Phat*~H*Inverse(S);
  xhat += K * nu;
  BLA::Matrix<1,1> I = { 1.0f };
  _Phat = (I - K*H)*_Phat;
}
