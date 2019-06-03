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
        static CalibrationMatrices Shah(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);
        static CalibrationMatrices Li(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);
        static CalibrationMatrices Park(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);

        static CalibrationError estimateCalibrationError(   Transform3d prMb,
                                                            Transform3d eeMt,
                                                            std::vector<Transform3d> bMee,
                                                            std::vector<Transform3d> prMt);
};

}

#endif //ROMOCC_CALIBRATIONMETHODS_H
