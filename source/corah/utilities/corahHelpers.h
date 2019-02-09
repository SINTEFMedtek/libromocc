#include "corahHeaders.h"

namespace corah
{

Eigen::Affine3d KDLFrameToEigenAffine(KDL::Frame frame);
Eigen::Affine3d ScaleTranslationAffine(Eigen::Affine3d affine, double scale);
Eigen::Matrix<double,6,1> AffineToVector6d(Eigen::Affine3d affine);
Eigen::Vector3d AffineToAxisAngle(Eigen::Affine3d affine);

}