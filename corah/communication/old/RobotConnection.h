#ifndef CXUR5CONNECTION_H
#define CXUR5CONNECTION_H

#include "SocketConnection.h"

/**
 * Class that handles Robot TCP connection.
 *
 * \author Andreas Østvik, SINTEF
 *
 */

class RobotConnection : public SocketConnection
{
    Q_OBJECT

public:
    RobotConnection(QString address, int port);
    RobotConnection();
    ~RobotConnection();

    void setAddress(QString address, int port);
    bool isConnectedToRobot();

    bool sendMessage(QString message);

    void updateCurrentState(QByteArray buffer);

    void setProtocol(QString protocolname);

    int getPacketSize();

private slots:
    void internalDataAvailable();

signals:
    void stateChanged();
    void packetReceived();

private:
    bool waitForUpdate();
    bool isPotentialPacket(qint64 bytes, std::vector<int> validPacketSize= {577, 812, 1044});

    ConnectionInfo info;
};

#endif // CXUR5CONNECTION_H
