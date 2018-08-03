//
// Created by androst on 06.07.18.
//

#ifndef CORAH_CORAHHEADERS_H
#define CORAH_CORAHHEADERS_H

// Standard library
#include <memory>

// Qt
#include <QObject>

// Eigen
#include <Eigen/Dense>

// Orocos KDL
#include <chain.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolverpos_nr.hpp>
#include <chainiksolvervel_pinv.hpp>

// Corah utilities
#include <utilities/corahHelpers.h>

// Supported manipulators
typedef enum {UR5} Manipulator;
typedef enum {movej,movep,speedj,speedl,stopj,stopl} MotionType;

#endif //CORAH_CORAHHEADERS_H
