//
// Created by androst on 29.06.18.
//

#ifndef CORAH_COMMUNICATIONINTERFACE_H
#define CORAH_COMMUNICATIONINTERFACE_H

#include <QObject>

#include "robotics/RobotState.h"
#include "communication/Client.h"
#include "communication/MessageDecoder.h"
#include "communication/MessageEncoder.h"

typedef enum {UR5} Manipulator;

class CommunicationInterface : public QObject
{
    Q_OBJECT

public:
    CommunicationInterface();
    ~CommunicationInterface();

    void config_connection(QString host, int port);
    void set_communication_protocol(Manipulator manipulator);

    bool connectToRobot();

    void moveJoints(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t, double rad);

    bool sendMessage(QString message);

private slots:
    void decodePackage(QByteArray package);

signals:
    void stateChanged(RobotState state);

private:
    MessageEncoder* mEncoder;
    MessageDecoder* mDecoder;
    Client mClient;

    QString mHost;
    int mPort;
};


#endif //CORAH_COMMUNICATIONINTERFACE_H
