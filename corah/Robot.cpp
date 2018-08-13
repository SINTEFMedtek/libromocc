//
// Created by androst on 29.06.18.
//

#include <iostream>

#include "Robot.h"

Robot::Robot():
eeMt(Eigen::Affine3d::Identity()),
rMb(Eigen::Affine3d::Identity())
{
    connect(&mCommunicationInterface,&CommunicationInterface::stateChanged,this,&Robot::updateCurrentState);
}

Robot::~Robot()
{

}

void Robot::configure(Manipulator manipulator, QString host, int port)
{
    mCommunicationInterface.set_communication_protocol(manipulator);
    mCommunicationInterface.config_connection(host, port);

    mCurrentState.set_kdlchain(manipulator);
}

bool Robot::start()
{
    return mCommunicationInterface.connectToRobot();
}

bool Robot::isConnected()
{
    return mCommunicationInterface.isConnected();
}

bool Robot::disconnectFromRobot()
{
    return mCommunicationInterface.disconnectFromRobot();
}

void Robot::shutdown()
{
    mCommunicationInterface.shutdownRobot();
}

RobotState Robot::getCurrentState()
{
    return mCurrentState;
}

void Robot::updateCurrentState(JointState state)
{
    mCurrentState.set_jointState(state.jointConfiguration, state.jointVelocity, state.timestamp);
    emit stateUpdated();
}

void Robot::runMotionQueue(MotionQueue queue)
{
    mMotionQueue = queue;
    RobotMotion target = mMotionQueue.front();
    target.targetPose.translation() = target.targetPose.translation()/1000;
    this->move(MotionType::movep, target.targetPose, target.acceleration, target.velocity);

    connect(this, &Robot::stateUpdated, this, &Robot::waitForMove);
}

void Robot::stopRunMotionQueue()
{
    disconnect(this, &Robot::stateUpdated, this, &Robot::waitForMove);
    this->stopMove(MotionType::stopj, 1.0);
}

void Robot::waitForMove()
{
    RobotMotion target = mMotionQueue.front();

    double remainingDistance = ((mCurrentState.bMee*eeMt).translation()-target.targetPose.translation()).norm();

    if(remainingDistance<=target.blendRadius)
    {
        mMotionQueue.erase(mMotionQueue.begin());
        RobotMotion target = mMotionQueue.front();
    }

    if(target.motionType == MotionType::speedj)
    {
        Vector6d targetJointVelocity = calculateJointVelocity(target);
        this->move(target.motionType, targetJointVelocity, target.acceleration, 0.0, 5.0, 0.0); // vel and r not used
    }


    if(mMotionQueue.empty())
    {
        this->stopMove(MotionType::stopj, target.acceleration);
        disconnect(this, &Robot::stateUpdated, this, &Robot::waitForMove);
    }
}

Vector6d Robot::calculateJointVelocity(RobotMotion target)
{
    Vector3d tangent = (target.targetPose.translation()-(mCurrentState.bMee*eeMt).translation());
    tangent = tangent/tangent.norm();

    Vector3d velocity =  tangent*target.velocity/1000;

    Vector3d rtangent = AffineToAxisAngle(target.targetPose)-AffineToAxisAngle(mCurrentState.bMee*eeMt);
    rtangent = rtangent/rtangent.norm();
    Vector3d rvelocity = rvelocity*target.velocity/1000;

    Vector6d velocityEndEffector;
    velocityEndEffector << velocity(0),velocity(1),velocity(2), 0, 0, 0;

    Vector6d jointVelocity = mCurrentState.getJacobian().inverse()*velocityEndEffector;

    return jointVelocity;

}

void Robot::stopMove(MotionType type, double acc)
{
    mCommunicationInterface.stopMove(type, acc);
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