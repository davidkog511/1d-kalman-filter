#ifndef SE_SF_H
#define SE_SF_H

#include <BasicLinearAlgebra.h>
#include "mecotron.h" // Include MECOTRON header

// Initial state variance [rad^2] = (0.005 m / R_WHEEL)^2: the start distance is tape-measured
// and typed in ValueOut1, with about 5 mm uncertainty. Used by resetStateEstimator().
const float P0_STATE = 2.366864e-02f;
// Variance the gate resets P to on a re-sync [rad^2] ~ (0.10 m / R_WHEEL)^2: 10 cm, so the
// filter trusts the measurement again and catches up quickly.
const float P_RESYNC = 9.5f;

void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat, BLA::Matrix<1> &_Phat);
void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat );
void ResetGate();   // clear the counter of consecutive gated readings


#endif // SE_SF_H
