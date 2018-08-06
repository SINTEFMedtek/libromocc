#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <utilities/corahHeaders.h>

/**
 * Struct that holds robot state information.
 *
 * \ingroup org_custusx_robotinterface
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

    Eigen::Affine3d getTransformToJoint(int jointNr);

    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;

    Eigen::RowVectorXd operationalConfiguration;
    Eigen::Affine3d bMee;

    KDL::ChainFkSolverPos_recursive getFKSolver();
    KDL::ChainIkSolverPos_NR getIKSolver();

    double timestamp;

private:
    KDL::Chain mKDLChain;
    KDL::ChainFkSolverPos_recursive *mFKSolver;
    KDL::ChainIkSolverPos_NR *mIKSolver;
    KDL::ChainIkSolverVel_pinv *mIKSolverVel;

    Eigen::Affine3d transform_to_joint(Eigen::RowVectorXd jointConfig, int jointNr=-1);




};

#endif // ROBOTSTATE_H
