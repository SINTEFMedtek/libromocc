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
    mKDLChain = setupKDLChain(manipulator);

    KDL::JntArray q_min(mKDLChain.getNrOfJoints()), q_max(mKDLChain.getNrOfJoints());
    for (int i = 0; i < mKDLChain.getNrOfJoints(); i++) {
        q_min(i) = mJointMinimum[i];
        q_max(i) = mJointMaximum[i];
    }

    mFKSolver = std::shared_ptr<FKSolver>(new FKSolver(mKDLChain));
    mIKSolverVel = std::shared_ptr<IKVelSolver>(new IKVelSolver(mKDLChain));
    //mIKSolver = std::shared_ptr<IKSolver>(new IKSolver(mKDLChain, q_min, q_max, *mFKSolver, *mIKSolverVel, 5000, 1e-5));
    mIKSolver = std::shared_ptr<IKSolver>(new IKSolver(mKDLChain, 1e-5 * 1e3, 500, 1e-15 * 1e3));
    mJacSolver = std::shared_ptr<JacobianSolver>(new JacobianSolver(mKDLChain));
}

void RobotState::setDecoder(Manipulator manipulator) {
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
    auto target_pose = TransformUtils::kdl::fromAffine(transform);
    auto q_init = TransformUtils::kdl::fromVector6D(this->getJointConfig());
    auto q_target = KDL::JntArray(6);
    auto target_affine = TransformUtils::kdl::toAffine(target_pose);

    int status = mIKSolver->CartToJnt(q_init, target_pose, q_target);
    if(status == 0){
        return Vector6d(q_target.data);
    } else{
        if(status == -5)
            std::cout << "Bad config. Max iterations exceeded." << std::endl;
        else
            std::cout << "Bad config." << std::endl;
        return Vector6d(q_target.data);
    }
}

Transform3d RobotState::jointConfigToOperationalConfig(romocc::Vector6d jointConfig) {
    return transform_to_joint(jointConfig);
}

}