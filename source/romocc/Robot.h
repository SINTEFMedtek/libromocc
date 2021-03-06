//
// Created by androst on 29.06.18.
//

#ifndef ROMOCC_ROBOT_H
#define ROMOCC_ROBOT_H

#include <thread>

#include "romocc/core/Object.h"
#include "romocc/robotics/RobotState.h"
#include "romocc/robotics/RobotMotion.h"
#include "romocc/robotics/RobotCoordinateSystem.h"
#include "romocc/communication/CommunicationInterface.h"

namespace romocc
{

class ROMOCC_EXPORT Robot : public Object
{
    ROMOCC_OBJECT(Robot)

    public:
        void configure(Manipulator manipulator, const std::string& ip_address, const int& port);
        bool connect();
        bool disconnect();
        bool isConnected() const;
        void shutdown();

        RobotState::pointer getCurrentState() const;
        void addUpdateSubscription(std::function<void()> updateFunction);

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0, bool wait=false);
        void stopMove(MotionType type, double acc);
        void sendProgram(std::string program);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();

        RobotCoordinateSystem::pointer getCoordinateSystem(){ return mCoordinateSystem;};

        Robot();
        ~Robot();

    private:
        void waitForMove();

        SharedPointer<CommunicationInterface> mCommunicationInterface;
        void startSubscription(std::function<void()> updateSignal);

        SharedPointer<RobotCoordinateSystem> mCoordinateSystem;
        SharedPointer<RobotState> mCurrentState;
        MotionQueue mMotionQueue;
        std::unique_ptr<std::thread> mThread;
        bool mActiveSubscription = false;
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad, bool wait)
{
    mCommunicationInterface->move(type, target, acc, vel, t, rad);
    if(wait == true)
    {
        auto remainingDistance = TransformUtils::norm(mCurrentState->get_bMee(), target);
        while(remainingDistance > 0.005 || isnan(remainingDistance)){
            remainingDistance = TransformUtils::norm(mCurrentState->get_bMee(), target);
        }
    }
}



} // namespace romocc

#endif //ROMOCC_ROBOT_H
