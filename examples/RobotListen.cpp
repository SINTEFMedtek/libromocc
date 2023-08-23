#include <iostream>
#include <chrono>
#include <thread>
#include <cxxopts.hpp>

#include "romocc/Robot.h"
#include "romocc/utilities/MathUtils.h"

using namespace romocc;

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " --ip <IP address> --manipulator <Manipulator type> --sw_version <version>\n";
}

int main(int argc, char *argv[])
{
    cxxopts::Options options("Robot listener", "Example of listening to robot states");

    options.add_options()
            ("ip", "IP address", cxxopts::value<std::string>()->default_value("192.168.231.131"))
            ("manipulator", "Manipulator type", cxxopts::value<std::string>()->default_value("UR5"))
            ("sw_version", "Software version", cxxopts::value<std::string>()->default_value("3.15"))
            ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::string ip = result["ip"].as<std::string>();
    std::string manipulator = result["manipulator"].as<std::string>();
    std::string sw_version = result["sw_version"].as<std::string>();

    std::cout << "Connecting to " << manipulator << " -- IP: " << ip << " -- SW: " << sw_version << std::endl;
    auto robot = Robot::New();
    robot->configure(Manipulator(manipulator, sw_version), ip, 30003);

    if(robot->connect())
    {
        auto initialJointConfig = robot->getCurrentState()->getJointConfig();
        std::cout << initialJointConfig.transpose() << std::endl;

        double previousTime = 0;
        bool stop = false;
        std::thread processing_thread([&](){
            while(!stop){
                Vector6d jointConfig = robot->getCurrentState()->getJointConfig();
                Vector6d operationalConfig = robot->getCurrentState()->getOperationalConfig();
                double currentTime = robot->getCurrentState()->getTimestamp();

                if(currentTime > previousTime)
                {
                    std::cout << "Joint config: " << TransformUtils::radToDeg(jointConfig).transpose() << std::endl;
                    std::cout << "Operational config: " << operationalConfig.transpose() << "\n" << std::endl;
                    previousTime = robot->getCurrentState()->getTimestamp();
                }
            }
        });

        std::this_thread::sleep_for(std::chrono::seconds(1));
        stop = true;
        processing_thread.join();
    }

}