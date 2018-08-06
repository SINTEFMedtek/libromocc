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

bool Robot::disconnect()
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