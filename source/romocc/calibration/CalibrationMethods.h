//
// Created by androst on 10.08.18.
//

#ifndef ROMOCC_CALIBRATIONMETHODS_H
#define ROMOCC_CALIBRATIONMETHODS_H

#include "CalibrationHelpers.h"

namespace romocc
{

class ROMOCC_EXPORT CalibrationMethods {
    public:
        static CalibrationMatrices Shah(std::vector<Transform3d> A, std::vector<Transform3d> B);
        static CalibrationMatrices Li(std::vector<Transform3d> A, std::vector<Transform3d> B);
        static CalibrationMatrices Park(std::vector<Transform3d> A, std::vector<Transform3d> B);

        static CalibrationError estimateCalibrationError(Transform3d A, Transform3d B,
                std::vector<Transform3d> X, std::vector<Transform3d> Y);
};

}

#endif //ROMOCC_CALIBRATIONMETHODS_H
