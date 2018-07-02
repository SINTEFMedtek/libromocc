#ifndef CORAH_SOCKET_H
#define CORAH_SOCKET_H

#include <QObject>
#include <QAbstractSocket>
#include <QString>
#include "SocketConnection.h"

/**
 * Reimplement QTcpServer::incomingConnection() to
 * do nothing but emit a signal.
 *
 * Used internally by SocketConnection.
 */
class SingleConnectionTcpServer : public QTcpServer
{
    Q_OBJECT
public:
    SingleConnectionTcpServer(QObject* parent);
//	void setSocket(QPointer<Socket> socket);
signals:
    void incoming(qintptr socketDescriptor);
protected:
    void incomingConnection(qintptr socketDescriptor);
private:
//	QPointer<Socket> mSocket;
};


class SocketConnector : public QObject
{
    Q_OBJECT
public:
    virtual ~SocketConnector() {}
    //	SocketConnector(SocketConnection::ConnectionInfo info, QSocket* socket);

    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual CX_SOCKETCONNECTION_STATE getState() = 0;
    virtual SocketConnection::ConnectionInfo getInfo() const = 0;
signals:
    void stateChanged(CX_SOCKETCONNECTION_STATE);
};

class SocketClientConnector : public SocketConnector
{
public:
    SocketClientConnector(SocketConnection::ConnectionInfo info, QTcpSocket* socket);
    virtual ~SocketClientConnector();

    virtual void activate();
    virtual void deactivate();
    virtual CX_SOCKETCONNECTION_STATE getState();
    virtual SocketConnection::ConnectionInfo getInfo() const { return mInfo; }

private:
    void internalConnected();
    void internalDisconnected();

    SocketConnection::ConnectionInfo mInfo;
    QTcpSocket* mSocket;

};

class SocketServerConnector : public SocketConnector
{
    Q_OBJECT
public:
    SocketServerConnector(SocketConnection::ConnectionInfo info, QTcpSocket* socket);
    virtual ~SocketServerConnector();

    virtual void activate();
    virtual void deactivate();
    virtual CX_SOCKETCONNECTION_STATE getState();
    virtual SocketConnection::ConnectionInfo getInfo() const { return mInfo; }

private:
    bool startListen();
    void stopListen();
    void incomingConnection(qintptr socketDescriptor);
    QStringList getAllServerHostnames();

    SocketConnection::ConnectionInfo mInfo;
    QPointer<class SingleConnectionTcpServer> mServer;
    QTcpSocket* mSocket;
};

#endif //CORAH_SOCKET_H
