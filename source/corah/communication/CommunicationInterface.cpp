#include "CommunicationInterface.h"

#include "corah/manipulators/ur5/Ur5MessageEncoder.h"
#include "corah/manipulators/ur5/Ur5MessageDecoder.h"

namespace corah
{

CommunicationInterface::CommunicationInterface()
{
    connect(&mClient,&Client::packageReceived,this,&CommunicationInterface::decodePackage);
}

CommunicationInterface::~CommunicationInterface()
{
}

bool CommunicationInterface::connectToRobot()
{
    return mClient.requestConnect(mHost, mPort);
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

bool CommunicationInterface::sendMessage(QString message)
{
    message.append('\n');

    QByteArray buffer;
    buffer = buffer.append(message);

    return mClient.sendPackage(buffer);
}

void CommunicationInterface::config_connection(QString host, int port)
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

void CommunicationInterface::decodePackage(QByteArray package)
{
    JointState state;
    state = mDecoder->analyzeTCPSegment(package);

    if(state.timestamp>0)
        emit(stateChanged(state));
}

void CommunicationInterface::stopMove(MotionType typeOfStop, double acc)
{
    sendMessage(mEncoder->stopCommand(typeOfStop, acc));
}

}