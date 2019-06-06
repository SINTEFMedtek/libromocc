//
// Created by androst on 06.06.19.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "romocc/Robot.h"

using namespace romocc;

int main(int argc, char *argv[])
{
    auto print_message = []()
    {
        std::cout << "Test" << "\n";
    };

    auto ur5 = Robot::New();
    ur5->configure(UR5, "localhost", 30003);
    ur5->start();
    ur5->addUpdateSubscription(print_message);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto initialJointConfig = ur5->getCurrentState().jointConfiguration;
    std::cout << initialJointConfig << std::endl;

    double targetConfig[] = {190,-400, 150, 1.0,-3.0, 0};
    ur5->move(romocc::MotionType::movep, targetConfig, 100, 50);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << ur5->getCurrentState().jointConfiguration << std::endl;
    ur5->move(romocc::MotionType::movej, initialJointConfig, 50, 25);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << ur5->getCurrentState().jointConfiguration << std::endl;
}