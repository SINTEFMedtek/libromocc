//
// Created by androst on 06.07.18.
//

#ifndef ROMOCC_FORWARDDECLARATIONS_H
#define ROMOCC_FORWARDDECLARATIONS_H

#include "PrecompiledHeaders.h"

namespace romocc
{

// Supported manipulators
typedef enum {UR5} Manipulator;

// Robot motion
typedef enum {movej,movep,speedj,speedl,stopj,stopl} MotionType;
typedef std::vector<struct RobotMotion> MotionQueue;

// Eigen robot typedefs
typedef Eigen::Affine3d Transform3d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,1,6> RowVector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;

}

#endif //ROMOCC_FORWARDDECLARATIONS_H
