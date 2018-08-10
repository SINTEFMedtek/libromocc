//
// Created by androst on 10.08.18.
//

#ifndef CUSTUSX_CALIBRATIONMETHODS_H
#define CUSTUSX_CALIBRATIONMETHODS_H

#include "CalibrationHelpers.h"

class CalibrationMethods {
    public:
        static CalibrationMatrices Shah(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);
        static CalibrationMatrices Li(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);
        static CalibrationMatrices Park(std::vector<Transform3d> prMt, std::vector<Transform3d> bMee);

        static CalibrationError estimateCalibrationError(   Transform3d prMb,
                                                            Transform3d eeMt,
                                                            std::vector<Transform3d> bMee,
                                                            std::vector<Transform3d> prMt);
};


#endif //CUSTUSX_CALIBRATIONMETHODS_H
