#include "Robot.h"
#include <iostream>

namespace romocc
{

Robot::Robot():
eeMt(Eigen::Affine3d::Identity()),
rMb(Eigen::Affine3d::Identity())
{
    mCommunicationInterface = CommunicationInterface::New();
    mCommunicationInterface->registerObserver(this);
}


void Robot::configure(Manipulator manipulator, std::string host, int port)
{
    mCommunicationInterface->set_communication_protocol(manipulator);
    mCommunicationInterface->config_connection(host, port);
    mCurrentState.set_kdlchain(manipulator);
}

bool Robot::start()
{
    mCommunicationInterface->getNotifier()->setupNotifier(5557);
    return mCommunicationInterface->connectToRobot();
}

bool Robot::isConnected()
{
    return mCommunicationInterface->isConnected();
}

bool Robot::disconnectFromRobot()
{
    return mCommunicationInterface->disconnectFromRobot();
}

void Robot::shutdown()
{
    mCommunicationInterface->shutdownRobot();
}

RobotState Robot::getCurrentState()
{
    return mCurrentState;
}

void Robot::update()
{
    JointState state = mCommunicationInterface->getCurrentState();
    mCurrentState.set_jointState(state.jointConfiguration, state.jointVelocity, state.timestamp);
    mCommunicationInterface->getNotifier()->broadcastUpdate();
}

void Robot::runMotionQueue(MotionQueue queue)
{
    mMotionQueue = std::move(queue);
    RobotMotion target = mMotionQueue.front();
    target.targetPose.translation() = target.targetPose.translation()/1000;
    this->move(MotionType::movep, target.targetPose, target.acceleration, target.velocity);
}

void Robot::stopRunMotionQueue()
{
    this->stopMove(MotionType::stopj, 1.0);
}

void Robot::waitForMove()
{
    auto target = mMotionQueue.front();

    double remainingDistance = ((mCurrentState.bMee*eeMt).translation()-target.targetPose.translation()).norm();

    if(remainingDistance<=target.blendRadius)
    {
        mMotionQueue.erase(mMotionQueue.begin());
        target = mMotionQueue.front();
    }

    if(target.motionType == MotionType::speedj)
    {
        auto targetJointVelocity = RobotMotionUtils::calcJointVelocity(target.targetPose,
                mCurrentState.bMee*eeMt, mCurrentState.getJacobian(), target.velocity);
        this->move(target.motionType, targetJointVelocity, target.acceleration, 0.0, 5.0, 0.0); // vel and r not used
    }


    if(mMotionQueue.empty())
    {
        this->stopMove(MotionType::stopj, target.acceleration);
    }
}

void Robot::stopMove(MotionType type, double acc)
{
    mCommunicationInterface->stopMove(type, acc);
};


void Robot::set_eeMt(Eigen::Affine3d eeMt)
{
    this->eeMt = eeMt;
}

void Robot::set_rMb(Eigen::Affine3d rMb)
{
    this->rMb = rMb;
}

Transform3d Robot::get_rMt()
{
    return rMb*getCurrentState().bMee*eeMt;
}

Transform3d Robot::get_rMb()
{
    return rMb;
}

Transform3d Robot::get_eeMt()
{
    return eeMt;
}

Robot::~Robot()
{

}

}