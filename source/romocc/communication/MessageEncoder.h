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
        virtual std::string moveCommand(MotionType type, Eigen::Affine3d pose, double acc, double vel, double t, double rad) = 0;
        virtual std::string moveCommand(MotionType type, double target[], double acc, double vel, double t, double rad) = 0;
        virtual std::string stopCommand(MotionType type, double acc) = 0;
        virtual std::string shutdownCommand() = 0;

    protected:
        std::string format(const std::string& format, ...);
};

}

#endif //ROMOCC_MESSAGEENCODER_H
