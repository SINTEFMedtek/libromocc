#include <thread>
#include <string>
#include <iostream>

#include "CommunicationInterface.h"

#include "romocc/manipulators/ur5/Ur5MessageEncoder.h"
#include "romocc/manipulators/ur5/Ur5MessageDecoder.h"

namespace romocc
{

CommunicationInterface::CommunicationInterface()
{
    mClient = Client::New();
    mContext = mClient->getContext();

    mUpdateNotifier = UpdateNotifier::New();
    mUpdateNotifier->setContext(mContext);
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
    auto subscriber = zmq_socket(mContext, ZMQ_SUB);
    auto rc = zmq_connect(subscriber, "inproc://raw_buffer"); assert(rc == 0);
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