//
// Created by androst on 27.06.18.
//

#include <QtNetwork>
#include <QDateTime>

#include "Client.h"


Client::Client(QObject *parent)
        : QObject(parent)
        , mSocket(new QTcpSocket(this))
{
    connect(mSocket, &QIODevice::readyRead, this, &Client::readPackage);
    connect(mSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error), this, &Client::displayError);
}

void Client::setAddress(QString ip_address, int port)
{
    mHost = ip_address;
    mPort = port;
}

bool Client::isConnected()
{
    return mSocket->waitForConnected();
}

bool Client::requestConnect()
{
    qInfo() << "Trying to connect...";
    mSocket->abort();
    mSocket->connectToHost(mHost, mPort);

    if(isConnected())
    {
        qInfo() << "Connected to host";
    }

    return isConnected();
}

void Client::readPackage()
{
    int timeDiff = QDateTime::currentMSecsSinceEpoch()-mCurrentTimestamp;

    qDebug() << "Bytes available:" << mSocket->bytesAvailable() << timeDiff;
    mCurrentTimestamp = QDateTime::currentMSecsSinceEpoch();

    QByteArray buffer;
    buffer = mSocket->readAll();

    emit(packageReceived(buffer));
}

bool Client::sendPackage(QByteArray package)
{
    if(!mSocket->waitForConnected())
        return false;

    qint64 writtenBytes = mSocket->write(package);
    return writtenBytes>0;
}

void Client::displayError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
        case QAbstractSocket::RemoteHostClosedError:
            break;
        case QAbstractSocket::HostNotFoundError:
            qCritical() << "The host was not found. Please check the host name and port settings.";
            break;
        case QAbstractSocket::ConnectionRefusedError:
            qCritical() << "The connection was refused by the peer. Make sure the server is running, "
                           "and check that the host name and port settings are correct.";
            break;
        default:
            qCritical() << QString("The following error occurred: %1.").arg(mSocket->errorString());
    }
}