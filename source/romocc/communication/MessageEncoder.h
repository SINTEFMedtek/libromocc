#ifndef ROMOCC_MESSAGEENCODER_H
#define ROMOCC_MESSAGEENCODER_H
#define _CRT_NO_VA_START_VALIDATION

#include <Eigen/Dense>

#include "romocc/core/Object.h"

namespace romocc
{

class ROMOCC_EXPORT MessageEncoder : public Object
{
    public:
        virtual std::string moveCommand(MotionType type, Eigen::RowVectorXd jointConfig, double acc, double vel, double t, double rad) = 0;
        virtual std::string moveCommand(MotionType type, Eigen::RowVectorXd jointConfig, double acc, double vel, double t, double lookahead_time, double gain) = 0;
        virtual std::string moveCommand(MotionType type, Eigen::Affine3d pose, double acc, double vel, double t, double rad) = 0;
        virtual std::string moveCommand(MotionType type, double target[], double acc, double vel, double t, double rad) = 0;
        virtual std::string stopCommand(MotionType type, double acc) = 0;
        virtual std::string shutdownCommand() = 0;
        virtual std::string setConfigurableOutput(int pin, bool value) = 0;
        virtual std::string setDigitalOutput(int pin, bool value) = 0;
        virtual std::string setAnalogOutput(int pin, double value) = 0;
        virtual std::string setToolVoltage(int voltage) = 0;
        virtual std::string setToolOutput(int pin, bool value) = 0;

    protected:
        std::string format(const std::string& format, ...);
};

}

#endif //ROMOCC_MESSAGEENCODER_H
