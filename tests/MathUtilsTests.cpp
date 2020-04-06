#include "catch/catch.hpp"
#include <iostream>

#include "romocc/core/ForwardDeclarations.h"
#include "romocc/utilities/MathUtils.h"

namespace romocc
{

TEST_CASE("Convert from Vector6D to Affine to Vector6D", "[romocc][MathUtils]") {
    romocc::Vector6d vec;
    vec << 0, 0, 100, M_PI, -M_PI_2, 0;
    std::cout << vec << std::endl;

    auto affine = TransformUtils::Affine::toAffine3DFromVector6D(vec);
    std::cout << affine.matrix() << std::endl;
    std::cout << (affine*affine.inverse()).matrix() << std::endl;

    auto new_vec = TransformUtils::Affine::toVector6D(affine);
    auto identity = Eigen::Affine3d::Identity();
    std::cout << new_vec << std::endl;

    CHECK((affine*affine.inverse()).matrix().isIdentity(1e-6));
}

}