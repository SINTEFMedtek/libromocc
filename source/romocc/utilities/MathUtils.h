#include "romocc/core/ForwardDeclarations.h"

namespace romocc
{

struct TransformUtils{
    struct Affine{
        static Eigen::Vector3d toAxisAngle(Eigen::Affine3d affine);
        static Eigen::Matrix<double,6,1> toVector6D(Eigen::Affine3d affine);
        static Eigen::Affine3d scaleTranslation(Eigen::Affine3d affine, double scale);
    };

    struct kdl{
        static Eigen::Affine3d toAffine(KDL::Frame frame);
    };
};

//Eigen::Affine3d kdlFrameToEigenAffine(KDL::Frame frame);
//Eigen::Affine3d scaleTranslationAffine(Eigen::Affine3d affine, double scale);
//Eigen::Matrix<double,6,1> affineToVector6d(Eigen::Affine3d affine);
//Eigen::Vector3d affineToAxisAngle(Eigen::Affine3d affine);

}