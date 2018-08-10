#ifndef CALIBRATIONHELPERS_H
#define CALIBRATIONHELPERS_H

/**
* Implementation of a calibration helper class.
*
* \ingroup org_custusx_robot_ur5
*
* \author Andreas Østvik
*/

#include <utilities/corahHeaders.h>

struct CalibrationError{
    double rotationError;
    double translationError;
    double rotStd;
    double transStd;
};

struct CalibrationMatrices{
    Transform3d prMb;
    Transform3d eeMt;
};

std::vector<Transform3d> invert_matrices(std::vector<Transform3d> matrices);
std::vector<double> compute_linspace(double start, double end, int steps);
double compute_average(std::vector<double> const &v);
double compute_variance(std::vector<double> const &v, double mean);
double sgn(double val);

#endif //CALIBRATIONHELPERS_H
