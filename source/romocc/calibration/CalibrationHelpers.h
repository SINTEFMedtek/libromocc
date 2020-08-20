#ifndef CALIBRATIONHELPERS_H
#define CALIBRATIONHELPERS_H

#include "romocc/core/ForwardDeclarations.h"
#include "romocc/utilities/MathUtils.h"

namespace romocc
{

/**
* Implementation of a calibration helper class.
*
* \author Andreas Østvik
*/


struct ROMOCC_EXPORT CalibrationError{
    double rotationError;
    double translationError;
    double rotStd;
    double transStd;
};

struct ROMOCC_EXPORT CalibrationMatrices{
    Transform3d prMb;
    Transform3d eeMt;
};

std::vector<Transform3d> invert_matrices(std::vector<Transform3d> matrices);
std::vector<double> compute_linspace(double start, double end, int steps);
double compute_average(std::vector<double> const &v);
double compute_variance(std::vector<double> const &v, double mean);
double sgn(double val);
Transform3d load_calibration_file(std::string filepath);
void save_calibration_file(std::string filepath, Transform3d calibration_matrix);

}

#endif //CALIBRATIONHELPERS_H
