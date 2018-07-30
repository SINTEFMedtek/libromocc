#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <utilities/corahHeaders.h>

/**
 * Struct that holds UR5 robot information.
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 * \date 2015-07-10
 */


class RobotState
{

public:
    RobotState();
    ~RobotState();

    void set_kdlchain(Manipulator manipulator);
    void set_jointState(Eigen::RowVectorXd q, Eigen::RowVectorXd q_vel, double timestamp);

    Eigen::RowVectorXd mJointConfiguration;
    Eigen::RowVectorXd mJointVelocity;
    Eigen::Affine3d bTee;

    KDL::ChainFkSolverPos_recursive getFKSolver();
    KDL::ChainIkSolverPos_NR getIKSolver();

private:
    KDL::Chain mKDLChain;
    KDL::ChainFkSolverPos_recursive *mFKSolver;
    KDL::ChainIkSolverPos_NR *mIKSolver;
    KDL::ChainIkSolverVel_pinv *mIKSolverVel;

    Eigen::Affine3d transform_to_joint(Eigen::RowVectorXd jointConfig, int jointNr=-1);

    double mTimestamp;



};

#endif // ROBOTSTATE_H
