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

    public:
        Robot();
        ~Robot();

        void configure(Manipulator manipulator, std::string ip_address, int port);
        bool start();
        bool isConnected();
        bool disconnectFromRobot();
        void shutdown();

        RobotState getCurrentState();

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0);
        void stopMove(MotionType type, double acc);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();

        Transform3d get_rMt();
        Transform3d get_rMb();
        Transform3d get_eeMt();
        void set_eeMt(Eigen::Affine3d eeMt);
        void set_rMb(Eigen::Affine3d rMb);

    private:
        void update();
        void waitForMove();

        SharedPointer<CommunicationInterface> mCommunicationInterface;

        RobotState mCurrentState;
        MotionQueue mMotionQueue;

        Transform3d eeMt, rMb;

        Vector6d calculateJointVelocity(RobotMotion target);
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad)
{
    mCommunicationInterface->move(type, target, acc, vel, t, rad);
};

}

#endif //ROMOCC_ROBOT_H
