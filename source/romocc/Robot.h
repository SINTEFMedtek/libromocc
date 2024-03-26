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
        connectionConfiguration getCommunicationConfiguration() const;

        void addUpdateSubscription(std::function<void()> updateFunction);

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0,  bool wait=false);

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t, double lookahead_time, double gain);

        void stopMove(MotionType type, double acc);
        void sendProgram(std::string program);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();
        void setConfigurableOutput(int pin, bool value);
        void setDigitalOutput(int pin, bool value);
        void setAnalogOutput(int pin, double value);
        void setToolVoltage(int voltage);
        void setToolOutput(int pin, bool value);

        RobotCoordinateSystem::pointer getCoordinateSystem(){ return mCoordinateSystem;};

        std::chrono::steady_clock::time_point currentTime();
        void wait(const std::chrono::steady_clock::time_point &t_cycle_start, double dt = 1/125.0);
        static void sleep(double seconds);

        Robot();
        ~Robot();

    private:
        void waitForMove();

        std::shared_ptr<CommunicationInterface> mCommunicationInterface;
        void startSubscription(std::function<void()> updateSignal);

        std::shared_ptr<RobotCoordinateSystem> mCoordinateSystem;
        std::shared_ptr<RobotState> mCurrentState;
        MotionQueue mMotionQueue;
        std::unique_ptr<std::thread> mThread;
        bool mActiveSubscription = false;
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad, bool wait)
{
    mCommunicationInterface->move(type, target, acc, vel, t, rad);
    if(wait)
    {
        double distanceThreshold = 0.005;
        const int timeoutInSeconds = 10; // Set your desired timeout in seconds
        const auto startTime = std::chrono::steady_clock::now();

        double remainingDistance;
        if(type == MotionType::movej){
            auto targetj = target.matrix().reshaped(6,1);
            remainingDistance = TransformUtils::norm(mCurrentState->getJointConfig(), targetj);
        } else {
            distanceThreshold = 0.015;
            remainingDistance = TransformUtils::norm(mCurrentState->get_bMee(), target);
        };

        while(remainingDistance > distanceThreshold || isnan(remainingDistance))
        {
            if(type == MotionType::movej){
                auto targetj = target.matrix().reshaped(6,1);
                remainingDistance = TransformUtils::norm(mCurrentState->getJointConfig(), targetj);
            } else {
                remainingDistance = TransformUtils::norm(mCurrentState->get_bMee(), target);
            };
            const auto currentTime = std::chrono::steady_clock::now();
            const auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
            if (elapsedTime > timeoutInSeconds)
            {
                break;
            }
        }
    }
}

template <>
void Robot::move<double*>(MotionType type, double* target, double acc, double vel, double t, double rad, bool wait)
{
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>> targetVector(target, 6);
    move(type, targetVector, acc, vel, t, rad, wait);
}

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double lookahead_time, double gain)
{
    mCommunicationInterface->move(type, target, acc, vel, t, lookahead_time, gain);
}

} // namespace romocc

#endif //ROMOCC_ROBOT_H
