//
// Created by androst on 29.06.18.
//

#include "CommunicationInterface.h"

CommunicationInterface::CommunicationInterface()
{

}

CommunicationInterface::~CommunicationInterface()
{

}

bool CommunicationInterface::connectToRobot(QString host, int port)
{
    mClient.setAddress(host, port);
    mClient.requestConnect();
}

bool CommunicationInterface::sendMessage(QString message)
{
    message.append('\n');

    QByteArray buffer;
    buffer = buffer.append(message);

    return mClient.sendPackage(buffer);
}
