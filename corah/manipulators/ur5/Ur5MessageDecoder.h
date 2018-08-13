#ifndef UR5RECEIVE_H
#define UR5RECEIVE_H

#include <QObject>

#include "robotics/RobotState.h"
#include "communication/MessageDecoder.h"

namespace corah
{

/**
 * Class that handles UR5 robot recieved messages.
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 *
 */

class CORAH_EXPORT Ur5MessageDecoder : public MessageDecoder
{
public:
    virtual JointState analyzeTCPSegment(QByteArray packet);

private:
    JointState setRTState(QByteArray data);

    Eigen::RowVectorXd getJointPositionsRT(QByteArray data);
    Eigen::RowVectorXd getJointVelocitiesRT(QByteArray data);
};

}

#endif // UR5RECEIVE_H
