//
// Created by androst on 29.06.18.
//

#include <iostream>

#include "Robot.h"

Robot::Robot()
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

RobotState Robot::getCurrentState()
{
    return mCurrentState;
}

void Robot::updateCurrentState(JointState state)
{
    mCurrentState.set_jointState(state.jointConfiguration, state.jointVelocity, state.timestamp);

    std::cout << mCurrentState.bTee.matrix() << std::endl << std::endl;
    std::cout << AffineToRowVector(mCurrentState.bTee) << std::endl << std::endl;
}

void Robot::move(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t, double rad)
{
    mCommunicationInterface.moveJoints(jointConfiguration, acc, vel, t, rad);
}


