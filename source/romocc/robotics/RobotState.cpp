#include <mutex>
#include "RobotState.h"
#include <iostream>
#include "romocc/manipulators/ur/UrKDLDefinition.h"
#include "romocc/manipulators/ur/UrMessageDecoder.h"


namespace romocc {

RobotState::RobotState() :
        mFKSolver(NULL),
        mIKSolverVel(NULL),
        mIKSolver(NULL),
        mJacSolver(NULL) {
}

void RobotState::unpack(uint8_t *buffer) {
    mValueLock.lock();
    JointState jointState = mDecoder->analyzeTCPSegment(buffer);
    this->setState(jointState.jointConfiguration, jointState.jointVelocity, jointState.timestamp);
    mValueLock.unlock();
}

void RobotState::setManipulator(romocc::Manipulator manipulator) {
    this->setKDLchain(manipulator);
    this->setDecoder(manipulator);
}


void RobotState::setKDLchain(Manipulator manipulator) {
    if (manipulator.manipulator == Manipulator::UR5) {
        mKDLChain = Ur5::KDLChain();
    } else if (manipulator.manipulator == Manipulator::UR10) {
        mKDLChain = Ur10::KDLChain();
    }

    KDL::JntArray q_min(mKDLChain.getNrOfJoints()), q_max(mKDLChain.getNrOfJoints());
    for (int i = 0; i < mKDLChain.getNrOfJoints(); i++) {
        q_min(i) = -2 * M_PI;
        q_max(i) = 2 * M_PI;
    }

    mFKSolver = std::shared_ptr<FKSolver>(new FKSolver(mKDLChain));
    mIKSolverVel = std::shared_ptr<IKVelSolver>(new IKVelSolver(mKDLChain));
    mIKSolver = std::shared_ptr<IKSolver>(new IKSolver(mKDLChain, q_min, q_max, *mFKSolver, *mIKSolverVel, 100, 1e-6));
    mJacSolver = std::shared_ptr<JacobianSolver>(new JacobianSolver(mKDLChain));
}

void RobotState::setDecoder(Manipulator manipulator) {
    if (manipulator.manipulator == Manipulator::UR5 || manipulator.manipulator == Manipulator::UR10)
        mDecoder = UrMessageDecoder::New();
}

void RobotState::setState(RowVector6d jointConfig, RowVector6d jointVel, double timestamp) {
    mTimestamp = timestamp;
    mJointConfiguration = jointConfig;
    mJointVelocity = jointVel;
    m_bMee = transform_to_joint(jointConfig);
    mOperationalConfiguration = TransformUtils::Affine::toVector6D(m_bMee);
}

Transform3d RobotState::transform_to_joint(RowVector6d jointConfig, int jointNr) {
    KDL::Frame output_T;

    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    for (unsigned int i = 0; i < mKDLChain.getNrOfJoints(); i++) { input_q(i) = jointConfig[i]; }

    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Transform3d transform = TransformUtils::kdl::toAffine(output_T);
    return TransformUtils::Affine::scaleTranslation(transform, 1000);
}

Matrix6d RobotState::getJacobian(int jointNr) {
    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    Vector6d jointConfig = getJointConfig();

    for (unsigned int i = 0; i < mKDLChain.getNrOfJoints(); i++) { input_q(i) = jointConfig[i]; }

    KDL::Jacobian output_jac(mKDLChain.getNrOfJoints());
    mJacSolver->JntToJac(input_q, output_jac, jointNr);

    return output_jac.data;
}

Eigen::Affine3d RobotState::getTransformToJoint(int jointNr) {
    KDL::Frame output_T;
    KDL::JntArray input_q(mKDLChain.getNrOfJoints());
    Vector6d jointConfig = getJointConfig();

    for (unsigned int i = 0; i < mKDLChain.getNrOfJoints(); i++) { input_q(i) = jointConfig[i]; }
    mFKSolver->JntToCart(input_q, output_T, jointNr);

    Eigen::Affine3d transform = TransformUtils::kdl::toAffine(output_T);
    return TransformUtils::Affine::scaleTranslation(transform, 1000);
}

std::shared_ptr<IKSolver> RobotState::getIKSolver() {
    return mIKSolver;
}

std::shared_ptr<FKSolver> RobotState::getFKSolver() {
    return mFKSolver;
}

Vector6d RobotState::getJointConfig() {
    Vector6d ret;
    mValueLock.lock();
    ret = mJointConfiguration;
    mValueLock.unlock();
    return ret;
}

Vector6d RobotState::getJointVelocity() {
    Vector6d ret;
    mValueLock.lock();
    ret = mJointVelocity;
    mValueLock.unlock();
    return ret;
}

Vector6d RobotState::getOperationalConfig() {
    Vector6d ret;
    mValueLock.lock();
    ret = mOperationalConfiguration;
    mValueLock.unlock();
    return ret;
}

Transform3d RobotState::get_bMee() {
    Transform3d ret;
    mValueLock.lock();
    ret = m_bMee;
    mValueLock.unlock();
    return ret;
}

double RobotState::getTimestamp() {
    double ret;
    mValueLock.lock();
    ret = mTimestamp;
    mValueLock.unlock();
    return ret;
}

Vector6d RobotState::getOperationalVelocity() {
    Vector6d ret;
    Matrix6d jacobian = getJacobian();
    mValueLock.lock();
    ret = jacobian * mJointVelocity;
    mValueLock.unlock();
    return ret;
}

Vector6d RobotState::operationalConfigToJointConfig(Transform3d transform) {
    auto ik_solver = this->getIKSolver();
    auto target_pose = TransformUtils::kdl::fromAffine(transform);
    auto q_current = TransformUtils::kdl::fromVector6D(this->getJointConfig());
    auto q_target = KDL::JntArray(6);

    int status = ik_solver->CartToJnt(q_current, target_pose, q_target);
    return Vector6d(q_target.data);
}

}