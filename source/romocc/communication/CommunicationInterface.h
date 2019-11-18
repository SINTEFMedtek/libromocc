#ifndef ROMOCC_COMMUNICATIONINTERFACE_H
#define ROMOCC_COMMUNICATIONINTERFACE_H

#include <thread>

#include "romocc/core/Object.h"
#include <romocc/utilities/ZMQUtils.h>
#include "romocc/robotics/RobotState.h"

#include "Client.h"
#include "MessageEncoder.h"


namespace romocc
{

class ROMOCC_EXPORT CommunicationInterface : public Object
{
    ROMOCC_OBJECT(CommunicationInterface)

    public:
        CommunicationInterface();
        ~CommunicationInterface();

        void configConnection(std::string host, int port);
        void setManipulator(Manipulator manipulator);

        RobotState::pointer getRobotState();

        bool connectToRobot();
        bool isConnected();
        bool disconnectFromRobot();
        void shutdownRobot();
        bool sendMessage(std::string message);

        template <class TargetConfiguration>
        void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);
        void stopMove(MotionType typeOfStop, double acc);

private:
        SharedPointer<Client> mClient;
        SharedPointer<MessageEncoder> mEncoder;
        RobotState::pointer mCurrentState;

        std::string mHost;
        int mPort;

        std::unique_ptr<std::thread> mThread;
        bool mStopThread;

        void decodeReceivedPackages();

};

template <class TargetConfiguration>
void CommunicationInterface::move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad)
{
    sendMessage(mEncoder->moveCommand(typeOfMotion, target, acc, vel, t, rad));
};

}

#endif //ROMOCC_COMMUNICATIONINTERFACE_H
