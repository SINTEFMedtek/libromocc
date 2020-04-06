#include <string>
#include <iostream>

#include "CommunicationInterface.h"

#include "romocc/manipulators/ur/UrMessageEncoder.h"
#include "romocc/manipulators/ur/UrMessageDecoder.h"
#include <romocc/utilities/ZMQUtils.h>


namespace romocc
{

CommunicationInterface::CommunicationInterface()
{
    mClient = Client::New();
    mCurrentState = RobotState::New();
}

CommunicationInterface::~CommunicationInterface()
{
    if(!mStopThread && this->isConnected()){
        mStopThread = true;
        mThread->join();
    }
}

bool CommunicationInterface::connectToRobot() {
    bool connected = mClient->requestConnect(mHost, mPort);
    if(connected)
    {
        mStopThread = false;
        mThread = std::make_unique<std::thread>(std::bind(&CommunicationInterface::decodeReceivedPackages, this));
    }
    return connected;
}

void CommunicationInterface::decodeReceivedPackages()
{
    auto subscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
    auto rc = zmq_connect(subscriber, "inproc://raw_buffer");
    assert(rc == 0);
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    auto notifier = ZMQUpdateNotifier("state_update_notifier");

    const uint16_t bufferSize = 2048;
    uint8_t buffer[bufferSize];
    bzero(buffer, bufferSize);

    while(!mStopThread)
    {
        rc = zmq_recv(subscriber, buffer, bufferSize, 0);
        if(rc>=764 || rc <=1116)
        {
            mCurrentState->unpack(buffer);
            notifier.broadcastUpdate("state_updated");
        }
    }
    notifier.close();
    zmq_close(subscriber);
}

bool CommunicationInterface::isConnected()
{
    return mClient->isConnected();
}

bool CommunicationInterface::disconnectFromRobot()
{
    mStopThread = true;
    mThread->join();
    bool disconnected = mClient->requestDisconnect();
    return disconnected;
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

void CommunicationInterface::configConnection(std::string host, int port)
{
    mHost = host;
    mPort = port;
}

void CommunicationInterface::setManipulator(romocc::Manipulator manipulator)
{
    auto encoder = UrMessageEncoder::New();
    encoder->setSoftwareVersion(manipulator.sw_version);

    mEncoder = encoder;
    mCurrentState->setManipulator(manipulator);
}

RobotState::pointer CommunicationInterface::getRobotState()
{
    return mCurrentState;
}

void CommunicationInterface::stopMove(MotionType typeOfStop, double acc)
{
    sendMessage(mEncoder->stopCommand(typeOfStop, acc));
}


}