//
// Created by androst on 06.07.18.
//

#ifndef CORAH_CORAHHEADERS_H
#define CORAH_CORAHHEADERS_H

// Standard library
#include <memory>

// Eigen
#include <Eigen/Dense>

// Orocos KDL
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Corah utilities
#include "corahExport.hpp"
#include "corahHelpers.h"

namespace corah
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

#endif //CORAH_CORAHHEADERS_H
