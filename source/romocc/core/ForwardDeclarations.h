//
// Created by androst on 06.07.18.
//

#ifndef ROMOCC_FORWARDDECLARATIONS_H
#define ROMOCC_FORWARDDECLARATIONS_H

#include "PrecompiledHeaders.h"

namespace romocc
{

// Supported manipulators
struct ROMOCC_EXPORT Manipulator{
    enum ManipulatorType {UR5, UR10};
    ManipulatorType manipulator;
    std::string sw_version;

    Manipulator(ManipulatorType manipulator = UR5, std::string sw_version = "3.0"){
        this->manipulator = manipulator;
        this->sw_version = sw_version;};
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
typedef KDL::ChainIkSolverPos_NR IKSolver;
typedef KDL::ChainIkSolverVel_pinv IKVelSolver;
typedef KDL::ChainJntToJacSolver JacobianSolver;


}

#endif //ROMOCC_FORWARDDECLARATIONS_H
