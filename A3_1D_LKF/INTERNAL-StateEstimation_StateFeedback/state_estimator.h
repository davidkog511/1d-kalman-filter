#ifndef SE_SF_H
#define SE_SF_H

#include <BasicLinearAlgebra.h>
#include "mecotron.h" // Include MECOTRON header

// Initial state variance [rad^2] = (0.10 m / R_WHEEL)^2: 10 cm initial uncertainty.
// Used by resetStateEstimator() and again when the gate re-syncs.
const float P0_STATE = 9.5f;

void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat, BLA::Matrix<1> &_Phat);
void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat );
void ResetGate();   // clear the counter of consecutive gated readings


#endif // SE_SF_H
