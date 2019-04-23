#include <QtNetwork>
#include <QDateTime>

#include "zmq.h"
#include "Client.h"

namespace corah
{

Client::Client(QObject *parent) : QObject(parent)
        , mSocket(new QTcpSocket(this))
{
    connect(mSocket, &QIODevice::readyRead, this, &Client::readPackage);
    //connect(mSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error), this, &Client::displayError);
    auto context = zmq_ctx_new();
    auto streamer = zmq_socket(context, ZMQ_STREAM);
    int rc = zmq_connect(streamer, "tcp://localhost:30003");
    uint8_t buffer[1044];
    uint8_t id [256];
    size_t  id_size = 256;

    rc = zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);

}

bool Client::isConnected()
{
    return mSocket->waitForConnected();
}

bool Client::requestConnect(QString host, int port)
{
    mConnectionInfo.host = host;
    mConnectionInfo.port = port;

    qInfo() << "Trying to connect...";
    mSocket->abort();
    mSocket->connectToHost(host, port);

    if(isConnected())
    {
        qInfo() << "Connected to host";
    }

    return isConnected();
}

bool Client::requestDisconnect()
{
    mSocket->disconnectFromHost();
    return mSocket->waitForDisconnected();
}

void Client::readPackage()
{
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

}