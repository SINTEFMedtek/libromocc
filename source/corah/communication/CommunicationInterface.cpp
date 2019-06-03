#include <thread>
#include <string>
#include <iostream>

#include "CommunicationInterface.h"

#include "corah/manipulators/ur5/Ur5MessageEncoder.h"
#include "corah/manipulators/ur5/Ur5MessageDecoder.h"

namespace corah
{

CommunicationInterface::CommunicationInterface()
{
}

CommunicationInterface::~CommunicationInterface()
{
}

bool CommunicationInterface::connectToRobot() {
    bool connected = mClient.requestConnect(mHost, mPort);
    std::thread thread_(std::bind(&CommunicationInterface::decodeReceivedPackages, this));
    thread_.detach();
    return connected;
}

void CommunicationInterface::decodeReceivedPackages()
{
    void *ctx = zmq_ctx_new();
    void *subscriber = zmq_socket(ctx, ZMQ_SUB);
    zmq_connect(subscriber, "tcp://localhost:5556");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    unsigned char buffer[1044];

    while(true)
    {
        zmq_recv(subscriber, buffer, 1044, 0);
        decodePackage(buffer);
    }
}

bool CommunicationInterface::isConnected()
{
    return mClient.isConnected();
}

bool CommunicationInterface::disconnectFromRobot()
{
    return mClient.requestDisconnect();
}

void CommunicationInterface::shutdownRobot()
{
    sendMessage(mEncoder->shutdownCommand());
}

bool CommunicationInterface::sendMessage(std::string message)
{
    message.append("\n");
    return mClient.sendPackage(message);
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
        mEncoder = new Ur5MessageEncoder();
        mDecoder = new Ur5MessageDecoder();
    }
}

void CommunicationInterface::decodePackage(unsigned char* package)
{
    JointState state;
    state = mDecoder->analyzeTCPSegment(package);
    mCurrentState = state;
    notifyObservers();
}

void CommunicationInterface::stopMove(MotionType typeOfStop, double acc)
{
    sendMessage(mEncoder->stopCommand(typeOfStop, acc));
}

void CommunicationInterface::registerObserver(Object *observer) {
    mObservers.push_back(observer);
}

void CommunicationInterface::removeObserver(Object *observer) {
    auto iterator = std::find(mObservers.begin(), mObservers.end(), observer);

    if (iterator != mObservers.end()) {
        mObservers.erase(iterator);
    }
}

void CommunicationInterface::notifyObservers() {
    for (Object *observer : mObservers) {
        observer->update();
    }
}

}