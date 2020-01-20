#ifndef URMESSAGEENCODER_H
#define URMESSAGEENCODER_H

#include "romocc/communication/MessageEncoder.h"
#include "romocc/robotics/RobotState.h"

namespace romocc
{

/**
 * Class that handles encoding of UR messages.
 *
 * \author Andreas Østvik
 *
 */

class ROMOCC_EXPORT UrMessageEncoder : public MessageEncoder
{
    ROMOCC_OBJECT(UrMessageEncoder)

    public:
        virtual std::string moveCommand(MotionType type, Eigen::RowVectorXd targetConfiguration, double acc, double vel, double t, double rad);
        virtual std::string moveCommand(MotionType type, Eigen::Affine3d targetPose, double acc, double vel, double t, double rad);
        virtual std::string moveCommand(MotionType type, double targetConfig[6], double acc, double vel, double t, double rad);

        virtual std::string stopCommand(MotionType typeOfStop, double acc);
        virtual std::string shutdownCommand();

        void setSoftwareVersion(std::string version);

private:
        std::string movej(Eigen::RowVectorXd jointConfig, double a, double v, double t, double r);
        std::string movep(Eigen::RowVectorXd operationalConfig, double a, double v, double t, double r);
        std::string movep(double operationalConfig[6], double a, double v, double t, double r);

        std::string speedj(Eigen::RowVectorXd jointVelocity, double a, double t);
        std::string speedl(Eigen::RowVectorXd operationalVelocity, double a, double t);

        std::string stopj(double a);
        std::string stopl(double a);

        std::string textmsg(std::string msg);
        std::string sleep(double time);

        std::string powerdown();

        std::string mURSoftwareVersion = "3.0";
        std::string mUrSeries = "CB-series";
        bool mNewSWVersion = false;
        bool compareVersions(std::string version_0, std::string version_1);
};

}

#endif // URMESSAGEENCODER_H
