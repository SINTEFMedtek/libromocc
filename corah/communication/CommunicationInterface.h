#ifndef CORAH_COMMUNICATIONINTERFACE_H
#define CORAH_COMMUNICATIONINTERFACE_H

#include "corah/utilities/corahHeaders.h"

#include "corah/robotics/RobotState.h"
#include "corah/communication/Client.h"
#include "corah/communication/MessageDecoder.h"
#include "corah/communication/MessageEncoder.h"

namespace corah
{

class CORAH_EXPORT CommunicationInterface : public QObject
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


    bool sendMessage(QString message);

    template <class TargetConfiguration>
    void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);
    void stopMove(MotionType typeOfStop, double acc);

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

}

#endif //CORAH_COMMUNICATIONINTERFACE_H
