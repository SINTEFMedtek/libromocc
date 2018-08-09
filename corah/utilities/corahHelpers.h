//
// Created by androst on 06.07.18.
//

#include <utilities/corahHeaders.h>

Eigen::Affine3d KDLFrameToEigenAffine(KDL::Frame frame);
Eigen::Affine3d ScaleTranslationAffine(Eigen::Affine3d affine, double scale);
Eigen::RowVectorXd AffineToRowVector(Eigen::Affine3d affine);
Eigen::Vector3d AffineToAxisAngle(Eigen::Affine3d affine);