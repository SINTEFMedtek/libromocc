#include <thread>
#include <string>
#include <iostream>
#include <romocc/utilities/ZMQUtils.h>

#include "CommunicationInterface.h"

#include "romocc/manipulators/ur5/Ur5MessageEncoder.h"
#include "romocc/manipulators/ur5/Ur5MessageDecoder.h"

namespace romocc
{

CommunicationInterface::CommunicationInterface()
{
    mClient = Client::New();
    mCurrentState = RobotState::New();
}

CommunicationInterface::~CommunicationInterface()
{
}

bool CommunicationInterface::connectToRobot() {
    bool connected = mClient->requestConnect(mHost, mPort);
    std::thread thread_(std::bind(&CommunicationInterface::decodeReceivedPackages, this));
    thread_.detach();
    return connected;
}

void CommunicationInterface::decodeReceivedPackages()
{
    auto subscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
    auto rc = zmq_connect(subscriber, "inproc://raw_buffer"); assert(rc == 0);
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    auto notifier = ZMQUpdateNotifier("state_update_notifier");

    unsigned char buffer[1044];

    while(true)
    {
        zmq_recv(subscriber, buffer, 1044, 0);
        updateState(buffer);
        notifier.broadcastUpdate("state_updated");
    }
}

bool CommunicationInterface::isConnected()
{
    return mClient->isConnected();
}

bool CommunicationInterface::disconnectFromRobot()
{
    return mClient->requestDisconnect();
}

void CommunicationInterface::shutdownRobot()
{
    sendMessage(mEncoder->shutdownCommand());
}

bool CommunicationInterface::sendMessage(std::string message)
{
    message.append("\n");
    return mClient->sendPackage(message);
}

void CommunicationInterface::config_connection(std::string host, int port)
{
    mHost = host;
    mPort = port;
}

void CommunicationInterface::set_communication_protocol(Manipulator manipulator)
{
    if(manipulator==UR5)
    {
        mEncoder = Ur5MessageEncoder::New();
        mDecoder = Ur5MessageDecoder::New();
    }
}

void CommunicationInterface::setRobotState(romocc::RobotState::pointer robotState)
{
    mCurrentState = robotState;
}

void CommunicationInterface::updateState(unsigned char* package)
{
    JointState jointState = mDecoder->analyzeTCPSegment(package);
    mCurrentState->setState(jointState.jointConfiguration, jointState.jointVelocity, jointState.timestamp);
}

void CommunicationInterface::stopMove(MotionType typeOfStop, double acc)
{
    sendMessage(mEncoder->stopCommand(typeOfStop, acc));
}


}