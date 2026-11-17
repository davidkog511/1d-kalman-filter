#include "state_estimator.h"


void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat,BLA::Matrix<1> &_Phat ) {
  
  // System F&B-matrix
  BLA::Matrix<1,1> F = { ?? };
  BLA::Matrix<1,1> B = { ?? };

  BLA::Matrix<1,1>Q = { ?? };
  BLA::Matrix<1,1>v = { ?? };

  // Evaluate discrete-time system dynamics
  xhat = F*xhat + B*v;
  _Phat = F*_Phat* ~F + Q;
}

void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat) {
  // Tuning parameter
  BLA::Matrix<1,1> R = {??};

  // Compute Jacobian of measurement equation
  BLA::Matrix<1,1> H = {??};

  // Compute innovation
  nu = y - H*xhat;

  // Set estimator gain
  BLA::Matrix<1,1> S = H * _Phat * ~H + R;
  BLA::Matrix<1,1> K = _Phat * ~H * Inverse(S);

  // Compute corrected system state estimate
  xhat += K * nu;
  BLA::Matrix<1,1> I = { 1.0f };
  _Phat = ( I- K*H)*_Phat; 
}
