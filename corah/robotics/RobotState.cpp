#include "RobotState.h"

#include "manipulators/ur5/Ur5KDLDefinition.h"

RobotState::RobotState() :
    mFKSolver(NULL),
    mIKSolverVel(NULL),
    mIKSolver(NULL),
    mJacSolver(NULL)
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
        mJacSolver = new KDL::ChainJntToJacSolver(mKDLChain);
    }
}

void RobotState::set_jointState(RowVector6d jointConfig, RowVector6d jointVel, double timestamp)
{
    timestamp = timestamp;
    jointConfiguration = jointConfig;
    jointVelocity = jointVel;

    bMee = transform_to_joint(jointConfiguration);
    operationalConfiguration = AffineToRowVector(bMee);
}

Transform3d RobotState::transform_to_joint(RowVector6d jointConfig, int jointNr)
{
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) {input_q(i)=jointConfig[i];}

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Transform3d transform = KDLFrameToEigenAffine(output_T);
    return ScaleTranslationAffine(transform, 1000);
}

Matrix6d RobotState::getJacobian(int jointNr)
{
    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) {input_q(i)=jointConfiguration[i];}

    KDL::Jacobian output_jac(mKDLChain.getNrOfJoints());
    mJacSolver->JntToJac(input_q, output_jac, jointNr);

    return output_jac.data;
}

Vector6d RobotState::getOperationalVelocity()
{
    return this->getJacobian()*jointVelocity;
}

Eigen::Affine3d RobotState::getTransformToJoint(int jointNr)
{
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) {input_q(i)=jointConfiguration[i];}

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Eigen::Affine3d transform = KDLFrameToEigenAffine(output_T);
    return ScaleTranslationAffine(transform, 1000);
}

KDL::ChainIkSolverPos_NR RobotState::getIKSolver()
{
    return *mIKSolver;
}

KDL::ChainFkSolverPos_recursive RobotState::getFKSolver()
{
    return *mFKSolver;
}
