#include "catch/catch.hpp"
#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include <vector>

#include "romocc/core/ForwardDeclarations.h"
#include "romocc/utilities/MathUtils.h"
#include "romocc/calibration/CalibrationMethods.h"
#include "romocc/calibration/CalibrationHelpers.h"


namespace romocc {

TEST_CASE("Dummy test Shah calibration", "[romocc][Calibration]") {
    romocc::Vector6d vec;
    vec << 0, 0, 100, M_PI, -M_PI_2, 0;
    auto affine = TransformUtils::Affine::toAffine3DFromVector6D(vec);

    std::vector<Transform3d> matStack;
    matStack.push_back(affine);
    matStack.push_back(affine);
    matStack.push_back(affine);

    auto calibMatrices = CalibrationMethods::Shah(matStack, matStack);
    std::cout << calibMatrices.X.matrix() << std::endl;
}

TEST_CASE("Load calibration file", "[romocc][Calibration]"){
    std::string filename = "tests/assets/calibration/cal_file.cal";
    auto cal_mat = load_calibration_file(filename);
    std::cout << cal_mat.matrix() << std::endl;
}

TEST_CASE("Save calibration files.", "[EchoBot][Utilities]") {
    auto path = "tests/assets/calibration/test.cal";

    auto calMat = Eigen::Affine3d::Identity();
    calMat(0,3) = 30.3;

    save_calibration_file(path, calMat);
}

}