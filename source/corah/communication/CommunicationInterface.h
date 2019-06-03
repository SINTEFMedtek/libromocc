#ifndef CORAH_COMMUNICATIONINTERFACE_H
#define CORAH_COMMUNICATIONINTERFACE_H

#include "corah/core/Object.h"
#include "corah/utilities/corahHeaders.h"
#include "corah/robotics/RobotState.h"

#include "Client.h"
#include "MessageDecoder.h"
#include "MessageEncoder.h"

namespace corah
{

class CORAH_EXPORT CommunicationInterface
{

public:
    CommunicationInterface();
    ~CommunicationInterface();

    void config_connection(std::string host, int port);
    void set_communication_protocol(Manipulator manipulator);

    bool connectToRobot();
    bool isConnected();
    bool disconnectFromRobot();
    void shutdownRobot();

    JointState getCurrentState(){return mCurrentState;};
    bool sendMessage(std::string message);

    template <class TargetConfiguration>
    void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);
    void stopMove(MotionType typeOfStop, double acc);

    void registerObserver(Object *observer);
    void removeObserver(Object*observer);

private:
    void decodePackage(unsigned char* package);
    void stateChanged();


    MessageEncoder *mEncoder;
    MessageDecoder *mDecoder;
    Client mClient;
    JointState mCurrentState;

    std::string mHost;
    int mPort;

    void decodeReceivedPackages();

    std::vector<Object *> mObservers;
    void notifyObservers();
};

template <class TargetConfiguration>
void CommunicationInterface::move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad)
{
    sendMessage(mEncoder->moveCommand(typeOfMotion, target, acc, vel, t, rad));
};

}

#endif //CORAH_COMMUNICATIONINTERFACE_H
