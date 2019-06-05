//
// Created by androst on 29.06.18.
//

#ifndef ROMOCC_ROBOT_H
#define ROMOCC_ROBOT_H

#include "romocc/core/Object.h"
#include "romocc/robotics/RobotState.h"
#include "romocc/robotics/RobotMotion.h"
#include "romocc/communication/CommunicationInterface.h"

namespace romocc
{

class ROMOCC_EXPORT Robot : public Object
{
    ROMOCC_OBJECT(Robot)

    public:
        Robot();
        ~Robot();

        void configure(Manipulator manipulator, const std::string& ip_address, const int& port);

        bool start();
        bool disconnectFromRobot();
        void shutdown();
        bool isConnected() const;

        RobotState getCurrentState() const;

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0);
        void stopMove(MotionType type, double acc);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();

        void updateSubscription(std::function<void()> updateSignal);


    Transform3d get_rMt();
        Transform3d get_rMb();
        Transform3d get_eeMt();
        void set_eeMt(Eigen::Affine3d eeMt);
        void set_rMb(Eigen::Affine3d rMb);

    private:
        void update() override;
        void waitForMove();

        SharedPointer<CommunicationInterface> mCommunicationInterface;
        void newSubscription(std::function<void()> updateSignal);

        RobotState mCurrentState;
        MotionQueue mMotionQueue;
        Transform3d eeMt, rMb;
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad)
{
    mCommunicationInterface->move(type, target, acc, vel, t, rad);
};

} // namespace romocc

#endif //ROMOCC_ROBOT_H
