#define _USE_MATH_DEFINES

#include <math.h>

#include "romocc/core/ForwardDeclarations.h"

namespace romocc
{

namespace TransformUtils{

    namespace Affine{
        ROMOCC_EXPORT Eigen::Vector3d toAxisAngle(Eigen::Affine3d affine);
        ROMOCC_EXPORT Eigen::Matrix<double,6,1> toVector6D(Eigen::Affine3d affine);
        ROMOCC_EXPORT Eigen::Affine3d toAffine3DFromVector6D(Eigen::Matrix<double,6,1> vector);
        ROMOCC_EXPORT Eigen::Affine3d scaleTranslation(Eigen::Affine3d affine, double scale);
    }

    namespace kdl{
        ROMOCC_EXPORT Eigen::Affine3d toAffine(KDL::Frame frame);
        ROMOCC_EXPORT KDL::Frame fromAffine(Eigen::Affine3d affine);
        ROMOCC_EXPORT KDL::JntArray fromVector6D(Eigen::Matrix<double,6,1> vector);
    }

    ROMOCC_EXPORT double norm(Vector6d a, Vector6d b);
    ROMOCC_EXPORT double norm(Transform3d a, Transform3d b);
    ROMOCC_EXPORT double norm(Transform3d a, Vector6d b);
    ROMOCC_EXPORT double norm(Transform3d a, double b[6]);

    ROMOCC_EXPORT Vector6d radToDeg(Vector6d vector);
}

} // namespace romocc