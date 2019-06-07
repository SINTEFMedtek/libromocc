#ifndef ROMOCC_COMMUNICATIONINTERFACE_H
#define ROMOCC_COMMUNICATIONINTERFACE_H

#include "romocc/core/Object.h"
#include <romocc/utilities/ZMQUtils.h>
#include "romocc/robotics/RobotState.h"

#include "Client.h"
#include "MessageDecoder.h"
#include "MessageEncoder.h"

namespace romocc
{

class ROMOCC_EXPORT CommunicationInterface : public Object
{
    ROMOCC_OBJECT(CommunicationInterface)

    public:
        CommunicationInterface();
        ~CommunicationInterface();

        void config_connection(std::string host, int port);
        void set_communication_protocol(Manipulator manipulator);

        bool connectToRobot();
        bool isConnected();
        bool disconnectFromRobot();
        void shutdownRobot();

        void setRobotState(RobotState::pointer robotState);
        bool sendMessage(std::string message);

        template <class TargetConfiguration>
        void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);
        void stopMove(MotionType typeOfStop, double acc);



private:
        void updateState(unsigned char* package);

        SharedPointer<Client> mClient;
        SharedPointer<MessageEncoder> mEncoder;
        SharedPointer<MessageDecoder> mDecoder;

        RobotState::pointer mCurrentState;
        std::string mHost;
        int mPort;

        void decodeReceivedPackages();

};

template <class TargetConfiguration>
void CommunicationInterface::move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad)
{
    sendMessage(mEncoder->moveCommand(typeOfMotion, target, acc, vel, t, rad));
};

}

#endif //ROMOCC_COMMUNICATIONINTERFACE_H
