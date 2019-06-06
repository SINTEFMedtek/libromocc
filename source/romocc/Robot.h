//
// Created by androst on 29.06.18.
//

#ifndef ROMOCC_ROBOT_H
#define ROMOCC_ROBOT_H

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

        bool start();
        bool disconnectFromRobot();
        bool isConnected() const;

        void shutdown();

        RobotState getCurrentState() const;
        void addUpdateSubscription(std::function<void()> updateFunction);

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0);
        void stopMove(MotionType type, double acc);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();

        RobotCoordinateSystem::pointer getCoordinateSystem(){ return mCoordinateSystem;};

        Robot();
        ~Robot();

    private:
        void update() override;
        void waitForMove();

        SharedPointer<CommunicationInterface> mCommunicationInterface;
        void startSubscription(std::function<void()> updateSignal);

        SharedPointer<RobotCoordinateSystem> mCoordinateSystem;
        RobotState mCurrentState;
        MotionQueue mMotionQueue;
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad)
{
    mCommunicationInterface->move(type, target, acc, vel, t, rad);
};



} // namespace romocc

#endif //ROMOCC_ROBOT_H
