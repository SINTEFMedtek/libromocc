//
// Created by androst on 27.06.18.
//

#ifndef CORAH_CLIENT_H
#define CORAH_CLIENT_H

#include <QTcpSocket>
#include <QByteArray>

class Client : public QObject
{
    Q_OBJECT

public:
    explicit Client(QObject *parent = nullptr);

    bool requestConnect(QString ip_address, int port);
    bool isConnected();

    bool sendPackage(QByteArray package);

    struct ConnectionInfo
    {
        QString host;
        int port;
    };

signals:
    void packageReceived(QByteArray package);

private slots:
    void readPackage();
    void displayError(QAbstractSocket::SocketError socketError);

private:
    ConnectionInfo mConnectionInfo;
    int mCurrentTimestamp;

    QTcpSocket* mSocket;
};


#endif //CORAH_CLIENT_H
