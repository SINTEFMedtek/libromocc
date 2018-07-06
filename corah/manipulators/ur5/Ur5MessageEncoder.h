#ifndef UR5MESSAGEENCODER_H
#define UR5MESSAGEENCODER_H

#include <QObject>

#include "communication/MessageEncoder.h"
#include "robotics/RobotState.h"

/**
 * Class that handles encoding of UR5 messages.
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 *
 */

class Ur5MessageEncoder : public MessageEncoder
{
public:
    virtual QString moveJoints(Eigen::RowVectorXd jointConfig, double acc, double vel, double t, double radius);
    virtual QString powerdown();

private:
    QString movej(Eigen::RowVectorXd pose, double a, double v, double t, double r);

    QString stopj(double a);
    QString stopl(double a);

    QString textmsg(QString msg);
    QString sleep(double time);
};

#endif // UR5MESSAGEENCODER_H
