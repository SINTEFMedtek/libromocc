#ifndef CORAH_MESSAGEENCODER_H
#define CORAH_MESSAGEENCODER_H

#include <QObject>
#include <Eigen/Dense>

#include "corah/utilities/corahHeaders.h"

namespace corah
{

class CORAH_EXPORT MessageEncoder {

public:
    virtual QString moveCommand(MotionType type, Eigen::RowVectorXd jointConfig, double acc, double vel, double t, double rad) = 0;
    virtual QString moveCommand(MotionType type, Eigen::Affine3d pose, double acc, double vel, double t, double rad) = 0;
    virtual QString stopCommand(MotionType type, double acc) = 0;
    virtual QString shutdownCommand() = 0;
};

}

#endif //CORAH_MESSAGEENCODER_H
