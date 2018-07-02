/*=========================================================================
This file is part of CustusX, an Image Guided Therapy Application.

Copyright (c) SINTEF Department of Medical Technology.
All rights reserved.

CustusX is released under a BSD 3-Clause license.

See Lisence.txt (https://github.com/SINTEFMedtek/CustusX/blob/master/License.txt) for details.
=========================================================================*/


#include "Socket.h"

#include <QTcpSocket>
#include <QTcpServer>
#include <QNetworkInterface>


SingleConnectionTcpServer::SingleConnectionTcpServer(QObject* parent) :
        QTcpServer(parent)
{
}


void SingleConnectionTcpServer::incomingConnection(qintptr socketDescriptor)
{
    emit incoming(socketDescriptor);
}

//---------------------------------------------------------
//---------------------------------------------------------
//---------------------------------------------------------

SocketClientConnector::SocketClientConnector(SocketConnection::ConnectionInfo info, QTcpSocket* socket) :
        mInfo(info),
        mSocket(socket)
{
    connect(mSocket, &QTcpSocket::connected, this, &SocketClientConnector::internalConnected);
    connect(mSocket, &QTcpSocket::disconnected, this, &SocketClientConnector::internalDisconnected);
}

SocketClientConnector::~SocketClientConnector()
{

}

void SocketClientConnector::activate()
{
    std::cout << "Trying to connect to " << mInfo.getDescription().toStdString() << std::endl;
    emit stateChanged(scsCONNECTING);
    mSocket->connectToHost(mInfo.host, mInfo.port);
}

void SocketClientConnector::deactivate()
{
    mSocket->close();
//	this->stopListen();
}

CX_SOCKETCONNECTION_STATE SocketClientConnector::getState()
{
    if (mSocket->state() == QAbstractSocket::ConnectedState)
        return scsCONNECTED;
    if (mSocket->state() == QAbstractSocket::UnconnectedState)
        return scsINACTIVE;
    return scsCONNECTING;
}

void SocketClientConnector::internalConnected()
{
    std::cout << "Connected to "  << mInfo.getDescription().toStdString() << std::endl;
    emit stateChanged(this->getState());
}

void SocketClientConnector::internalDisconnected()
{
    std::cout << "Disconnected" << std::endl;
    this->stateChanged(this->getState());
}


//---------------------------------------------------------
//---------------------------------------------------------
//---------------------------------------------------------

SocketServerConnector::SocketServerConnector(SocketConnection::ConnectionInfo info, QTcpSocket* socket) :
        mInfo(info),
        mSocket(socket)
{

}

SocketServerConnector::~SocketServerConnector()
{

}

void SocketServerConnector::activate()
{
    this->startListen();
}

void SocketServerConnector::deactivate()
{
    this->stopListen();
}

CX_SOCKETCONNECTION_STATE SocketServerConnector::getState()
{
    if (mSocket->state() == QAbstractSocket::ConnectedState)
        return scsCONNECTED;
    if (mServer->isListening())
        return scsLISTENING;
    if (mSocket->state() == QAbstractSocket::UnconnectedState)
        return scsINACTIVE;
    return scsCONNECTING;
}

bool SocketServerConnector::startListen()
{
    if (!mServer)
    {
        mServer = new SingleConnectionTcpServer(this);
        connect(mServer.data(), &SingleConnectionTcpServer::incoming, this, &SocketServerConnector::incomingConnection);
//		mServer->setSocket(mSocket);
    }
    emit stateChanged(scsCONNECTING);

    bool started = mServer->listen(QHostAddress::Any, mInfo.port);

    if (started)
    {
        std::cout << QString("Server address: %1").arg(this->getAllServerHostnames().join(", ")).toStdString();
        std::cout << QString("Server is listening to port %1").arg(mServer->serverPort()).toStdString();
    }
    else
    {
        std::cout << QString("Server failed to start. Error: %1").arg(mServer->errorString()).toStdString();
    }

    emit stateChanged(this->getState());
    return started;
}

void SocketServerConnector::stopListen()
{
    mSocket->close();

    if (mServer && mServer->isListening())
    {
        std::cout << QString("Server stopped listening to port %1").arg(mServer->serverPort()).toStdString();
        mServer->close();
        emit stateChanged(this->getState());
    }
}

void SocketServerConnector::incomingConnection(qintptr socketDescriptor)
{
    std::cout << "Server: Incoming connection...";

    if (this->mSocket->state() == QAbstractSocket::ConnectedState)
    {
        std::cout << "Incoming connection request rejected: The server can only handle a single connection.";
        return;
    }

    int success = mSocket->setSocketDescriptor(socketDescriptor, QAbstractSocket::ConnectedState);
    QString clientName = mSocket->localAddress().toString();
    std::cout << QString("Connected to "+ clientName+". Session started." + success).toStdString();

    emit stateChanged(this->getState());
}

QStringList SocketServerConnector::getAllServerHostnames()
{
    QStringList addresses;

            foreach(QNetworkInterface interface, QNetworkInterface::allInterfaces())
        {
            if (interface.flags().testFlag(QNetworkInterface::IsRunning))
                        foreach (QNetworkAddressEntry entry, interface.addressEntries())
                    {
                        if ( interface.hardwareAddress() != "00:00:00:00:00:00"
                             && entry.ip().toString() != "127.0.0.1"
                             && entry.ip().toString().contains(".") )
                            addresses << QString("%1: %2").arg(interface.name()).arg(entry.ip().toString());
                    }
        }

    return addresses;
}