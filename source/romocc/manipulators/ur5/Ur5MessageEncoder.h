#ifndef UR5MESSAGEENCODER_H
#define UR5MESSAGEENCODER_H

#include "romocc/communication/MessageEncoder.h"
#include "romocc/robotics/RobotState.h"

namespace romocc
{

/**
 * Class that handles encoding of UR5 messages.
 *
 * \author Andreas Østvik
 *
 */

class ROMOCC_EXPORT Ur5MessageEncoder : public MessageEncoder
{
    ROMOCC_OBJECT(Ur5MessageEncoder)

    public:
        virtual std::string moveCommand(MotionType type, Eigen::RowVectorXd targetConfiguration, double acc, double vel, double t, double rad);
        virtual std::string stopCommand(MotionType typeOfStop, double acc);
        virtual std::string shutdownCommand();

        std::string moveCommand(MotionType type, Eigen::Affine3d targetPose, double acc, double vel, double t, double rad);

    private:
        std::string movej(Eigen::RowVectorXd jointConfig, double a, double v, double t, double r);
        std::string movep(Eigen::RowVectorXd operationalConfig, double a, double v, double t, double r);

        std::string speedj(Eigen::RowVectorXd jointVelocity, double a, double t);
        std::string speedl(Eigen::RowVectorXd operationalVelocity, double a, double t);

        std::string stopj(double a);
        std::string stopl(double a);

        std::string textmsg(std::string msg);
        std::string sleep(double time);

        std::string powerdown();

};

}

#endif // UR5MESSAGEENCODER_H
