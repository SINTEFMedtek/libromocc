#include "RobotState.h"

#include "romocc/manipulators/ur5/Ur5KDLDefinition.h"

namespace romocc {

RobotState::RobotState() :
        mFKSolver(NULL),
        mIKSolverVel(NULL),
        mIKSolver(NULL),
        mJacSolver(NULL) {
}

RobotState::~RobotState() {
}

void RobotState::setKDLchain(Manipulator manipulator) {
    if (manipulator == UR5) {
        mKDLChain = Ur5Chain();
        mFKSolver = new KDL::ChainFkSolverPos_recursive(mKDLChain);
        mIKSolverVel = new KDL::ChainIkSolverVel_pinv(mKDLChain);
        mIKSolver = new KDL::ChainIkSolverPos_NR(mKDLChain, *mFKSolver, *mIKSolverVel, 100, 1e-6);
        mJacSolver = new KDL::ChainJntToJacSolver(mKDLChain);
    }
}

void RobotState::setJointState(RowVector6d jointConfig, RowVector6d jointVel, double timestamp) {
    mTimestamp = timestamp;
    mJointConfiguration = jointConfig;
    mJointVelocity = jointVel;
    m_bMee = transform_to_joint(jointConfig);
    mOperationalConfiguration = TransformUtils::Affine::toVector6D(m_bMee);
}

Transform3d RobotState::transform_to_joint(RowVector6d jointConfig, int jointNr) {
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) { input_q(i) = jointConfig[i]; }

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Transform3d transform = TransformUtils::kdl::toAffine(output_T);
    return TransformUtils::Affine::scaleTranslation(transform, 1000);
}

Matrix6d RobotState::getJacobian(int jointNr) const
{
    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) { input_q(i) = mJointConfiguration[i]; }

    KDL::Jacobian output_jac(mKDLChain.getNrOfJoints());
    mJacSolver->JntToJac(input_q, output_jac, jointNr);

    return output_jac.data;
}

Vector6d RobotState::getOperationalVelocity() const
{
    return this->getJacobian() * mJointVelocity;
}

Eigen::Affine3d RobotState::getTransformToJoint(int jointNr) const
{
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < 6; i++) { input_q(i) = mJointConfiguration[i]; }

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Eigen::Affine3d transform = TransformUtils::kdl::toAffine(output_T);
    return TransformUtils::Affine::scaleTranslation(transform, 1000);
}

KDL::ChainIkSolverPos_NR RobotState::getIKSolver() {
    return *mIKSolver;
}

KDL::ChainFkSolverPos_recursive RobotState::getFKSolver() {
    return *mFKSolver;
}

Vector6d RobotState::getJointConfig() const
{
    return mJointConfiguration;
}

Vector6d RobotState::getJointVelocity() const
{
    return mJointConfiguration;
}

Vector6d RobotState::getOperationalConfig() const
{
    return mJointConfiguration;
}

Transform3d RobotState::get_bMee() const
{
    return m_bMee;
}

}