#ifndef UR5MESSAGEENCODER_H
#define UR5MESSAGEENCODER_H

#include <QObject>

#include "communication/MessageEncoder.h"
#include "robotics/RobotState.h"

/**
 * Class that handles encoding of UR5 messages.
 *
 * \author Andreas Østvik
 *
 */

class Ur5MessageEncoder : public MessageEncoder
{
public:
    virtual QString moveCommand(MotionType type, Eigen::RowVectorXd targetConfiguration, double acc, double vel, double t, double rad);
    virtual QString stopCommand(MotionType typeOfStop, double acc);
    virtual QString shutdownCommand();

    QString moveCommand(MotionType type, Eigen::Affine3d targetPose, double acc, double vel, double t, double rad);

private:
    QString movej(Eigen::RowVectorXd jointConfig, double a, double v, double t, double r);
    QString movep(Eigen::RowVectorXd operationalConfig, double a, double v, double t, double r);

    QString speedj(Eigen::RowVectorXd jointVelocity, double a, double t);
    QString speedl(Eigen::RowVectorXd operationalVelocity, double a, double t);

    QString stopj(double a);
    QString stopl(double a);

    QString textmsg(QString msg);
    QString sleep(double time);

    QString powerdown();

};

#endif // UR5MESSAGEENCODER_H
