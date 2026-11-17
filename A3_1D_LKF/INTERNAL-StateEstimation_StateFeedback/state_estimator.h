#ifndef SE_SF_H
#define SE_SF_H

#include <BasicLinearAlgebra.h>
#include "mecotron.h" // Include MECOTRON header

void PredictionUpdate(const BLA::Matrix<2> &u, BLA::Matrix<1> &xhat, BLA::Matrix<1> &_Phat);
void CorrectionUpdate(const BLA::Matrix<1> &y, BLA::Matrix<1> &xhat, BLA::Matrix<1> &nu,BLA::Matrix<1> &_Phat );


#endif // SE_SF_H
