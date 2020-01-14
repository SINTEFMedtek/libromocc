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
    }
}

} // namespace romocc