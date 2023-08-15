//
// Created by androst on 06.07.18.
//

#ifndef ROMOCC_FORWARDDECLARATIONS_H
#define ROMOCC_FORWARDDECLARATIONS_H

#include "PrecompiledHeaders.h"

namespace romocc
{

// Supported manipulators
typedef enum {UR5, UR10, UR10e} ManipulatorType;
struct ROMOCC_EXPORT Manipulator{
    ManipulatorType manipulator;
    std::string sw_version;

    ManipulatorType manipulatorTypeFromString(const std::string& manipulator) {
        if (manipulator == "UR5") {
            return UR5;
        } else if (manipulator == "UR10") {
            return UR10;
        } else if (manipulator == "UR10e") {
            return UR10e;
        } else {
            throw std::runtime_error("Invalid manipulator type string");
        }
    }

    Manipulator(ManipulatorType manipulator = UR5, std::string sw_version = "3.0"){
        this->manipulator = manipulator;
        this->sw_version = sw_version;
    }

    Manipulator(const std::string& manipulator, std::string sw_version = "3.0"){
        this->manipulator = manipulatorTypeFromString(manipulator);
        this->sw_version = sw_version;
    }
};

// Robot motion
typedef enum {movej,movep,speedj,speedl,stopj,stopl} MotionType;
typedef std::vector<struct RobotMotion> MotionQueue;

// Eigen robot typedefs
typedef Eigen::Affine3d Transform3d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,1,6> RowVector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;

// KDL
typedef KDL::Chain RobotChain;
typedef KDL::ChainFkSolverPos_recursive FKSolver;
typedef KDL::ChainIkSolverPos_LMA IKSolver;
typedef KDL::ChainIkSolverVel_pinv IKVelSolver;
typedef KDL::ChainJntToJacSolver JacobianSolver;

// Struct packaging
#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

}

#endif //ROMOCC_FORWARDDECLARATIONS_H
