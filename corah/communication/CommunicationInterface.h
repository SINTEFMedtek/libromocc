//
// Created by androst on 29.06.18.
//

#ifndef CORAH_COMMUNICATIONINTERFACE_H
#define CORAH_COMMUNICATIONINTERFACE_H

#include <utilities/corahHeaders.h>

#include "robotics/RobotState.h"
#include "communication/Client.h"
#include "communication/MessageDecoder.h"
#include "communication/MessageEncoder.h"

class CommunicationInterface : public QObject
{
    Q_OBJECT

public:
    CommunicationInterface();
    ~CommunicationInterface();

    void config_connection(QString host, int port);
    void set_communication_protocol(Manipulator manipulator);

    bool connectToRobot();
    bool isConnected();
    bool disconnectFromRobot();
    void shutdownRobot();

    void stopMove(QString typeOfStop, double acc);

    bool sendMessage(QString message);

    template <class TargetConfiguration>
    void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);

private slots:
    void decodePackage(QByteArray package);

signals:
    void stateChanged(JointState state);

private:
    MessageEncoder *mEncoder;
    MessageDecoder *mDecoder;
    Client mClient;

    QString mHost;
    int mPort;
};

template <class TargetConfiguration>
void CommunicationInterface::move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad)
{
    sendMessage(mEncoder->moveCommand(typeOfMotion, target, acc, vel, t, rad));
};


#endif //CORAH_COMMUNICATIONINTERFACE_H
