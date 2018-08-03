#include "RobotState.h"

#include "manipulators/ur5/Ur5KDLDefinition.h"

RobotState::RobotState() :
    mFKSolver(NULL),
    mIKSolverVel(NULL),
    mIKSolver(NULL)
{
}

RobotState::~RobotState()
{
}

void RobotState::set_kdlchain(Manipulator manipulator)
{
    if(manipulator==UR5)
    {
        mKDLChain = Ur5Chain();
        mFKSolver = new KDL::ChainFkSolverPos_recursive(mKDLChain);
        mIKSolverVel = new KDL::ChainIkSolverVel_pinv(mKDLChain);
        mIKSolver = new KDL::ChainIkSolverPos_NR(mKDLChain, *mFKSolver, *mIKSolverVel, 100, 1e-6);
    }
}

void RobotState::set_jointState(Eigen::RowVectorXd jointConfig, Eigen::RowVectorXd jointVel, double timestamp)
{
    mTimestamp = timestamp;
    mJointConfiguration = jointConfig;
    mJointVelocity = jointVel;

    bTee = transform_to_joint(mJointConfiguration);
    operationalConfiguration = AffineToRowVector(bTee);
}

Eigen::Affine3d RobotState::transform_to_joint(Eigen::RowVectorXd jointConfig, int jointNr)
{
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) {input_q(i)=jointConfig[i];}

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    return KDLFrameToEigenAffine(output_T);
}

KDL::ChainIkSolverPos_NR RobotState::getIKSolver()
{
    return *mIKSolver;
}

KDL::ChainFkSolverPos_recursive RobotState::getFKSolver()
{
    return *mFKSolver;
}
