//
// Created by androst on 06.07.18.
//

#include <utilities/corahHeaders.h>

Eigen::Affine3d KDLFrameToEigenAffine(KDL::Frame frame);
Eigen::RowVectorXd AffineToRowVector(Eigen::Affine3d affine);

