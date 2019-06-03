#ifndef CORAH_MESSAGEENCODER_H
#define CORAH_MESSAGEENCODER_H

#include <Eigen/Dense>

#include "corah/utilities/corahHeaders.h"

namespace corah
{

class CORAH_EXPORT MessageEncoder {

public:
    virtual std::string moveCommand(MotionType type, Eigen::RowVectorXd jointConfig, double acc, double vel, double t, double rad) = 0;
    virtual std::string moveCommand(MotionType type, Eigen::Affine3d pose, double acc, double vel, double t, double rad) = 0;
    virtual std::string stopCommand(MotionType type, double acc) = 0;
    virtual std::string shutdownCommand() = 0;

protected:
    std::string format(const std::string& format, ...);
};

}

#endif //CORAH_MESSAGEENCODER_H
