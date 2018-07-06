#include "RobotState.h"

#include "manipulators/ur5/Ur5KDLDefinition.h"

RobotState::RobotState() :
mFKSolver(mKDLChain)
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
        KDL::ChainFkSolverPos_recursive mFKSolver(mKDLChain);
        //mIKSolver = new KDL::ChainIkSolverPos_LMA(mKDLChain);
    }
}

void RobotState::set_jointState(Eigen::RowVectorXd jointConfig, Eigen::RowVectorXd jointVel, double timestamp)
{
    mTimestamp = timestamp;
    mJointConfiguration = jointConfig;
    mJointVelocity = jointVel;

    bTee = transform_to_joint(mJointConfiguration, mKDLChain.getNrOfJoints());
}

Eigen::Affine3d RobotState::transform_to_joint(Eigen::RowVectorXd jointConfig, int jointNr)
{
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) {input_q(i)=jointConfig[i];}

    mFKSolver.JntToCart(input_q, output_T, jointNr);

    return KDLFrameToEigenAffine(output_T);
}


