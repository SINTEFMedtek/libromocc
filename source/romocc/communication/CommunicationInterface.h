#ifndef ROMOCC_COMMUNICATIONINTERFACE_H
#define ROMOCC_COMMUNICATIONINTERFACE_H

#include "romocc/core/Object.h"
#include "romocc/robotics/RobotState.h"

#include "Client.h"
#include "MessageDecoder.h"
#include "MessageEncoder.h"

namespace romocc
{

class ROMOCC_EXPORT CommunicationInterface
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

        JointState getCurrentState(){return mCurrentState;};
        bool sendMessage(std::string message);

        template <class TargetConfiguration>
        void move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad);
        void stopMove(MotionType typeOfStop, double acc);

        void registerObserver(Object *observer);
        void removeObserver(Object*observer);

        class UpdateNotifier
        {
            ROMOCC_OBJECT(UpdateNotifier)

            public:
                void setContext(void* context){mContext = context;};
                void setupNotifier(int port = 5557){
                    mPublisher = zmq_socket(mContext, ZMQ_PUB);
                    zmq_bind(mPublisher, ("tcp://*:" + std::to_string(port)).c_str());
                }

                void broadcastUpdate(std::string message = "State updated"){
                    zmq_msg_t zmqMessage;
                    zmq_msg_init_size(&zmqMessage, message.size());
                    memcpy(zmq_msg_data(&zmqMessage), message.c_str(), message.size());
                    zmq_sendmsg(mPublisher, &zmqMessage, ZMQ_DONTWAIT);
                };

            private:
                UpdateNotifier(){};

                void* mContext;
                void* mPublisher;
                std::weak_ptr<UpdateNotifier> mPtr;
        };

        UpdateNotifier::pointer getNotifier(){ return mUpdateNotifier;};

    private:
        void decodePackage(unsigned char* package);

        SharedPointer<Client> mClient;
        SharedPointer<MessageEncoder> mEncoder;
        SharedPointer<MessageDecoder> mDecoder;

        JointState mCurrentState;

        std::string mHost;
        int mPort;

        void decodeReceivedPackages();

        std::vector<Object *> mObservers;
        void notifyObservers();

        SharedPointer<UpdateNotifier> mUpdateNotifier;
        void* mContext;

        std::weak_ptr<CommunicationInterface> mPtr;
};

template <class TargetConfiguration>
void CommunicationInterface::move(MotionType typeOfMotion, TargetConfiguration target, double acc, double vel, double t, double rad)
{
    sendMessage(mEncoder->moveCommand(typeOfMotion, target, acc, vel, t, rad));
};

}

#endif //ROMOCC_COMMUNICATIONINTERFACE_H
