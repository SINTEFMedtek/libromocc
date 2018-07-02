//
// Created by androst on 29.06.18.
//

#include <iostream>
#include <Eigen/Dense>

#include "Robot.h"

Robot::Robot()
{
    connect(&mCommunicationInterface,&CommunicationInterface::stateChanged,this,&Robot::updateCurrentState);
}

Robot::~Robot()
{

}

void Robot::setup(Manipulator manipulator, QString host, int port)
{
    mCommunicationInterface.set_communication_protocol(manipulator);
    mCommunicationInterface.config_connection(host, port);
}

bool Robot::initialize()
{
    return mCommunicationInterface.connectToRobot();
}

RobotState Robot::getCurrentState()
{
    return mCurrentState;
}

void Robot::updateCurrentState(RobotState state)
{
    mCurrentState = state;
}

void Robot::move(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t, double rad)
{
    mCommunicationInterface.moveJoints(jointConfiguration, acc, vel, t, rad);
}
