#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <Eigen/Dense>

typedef std::vector<struct RobotMovementInfo> MovementQueue;

/**
 * Struct that holds UR5 robot information.
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 * \date 2015-07-10
 */


struct RobotState
{
    RobotState(bool updated = false);
    ~RobotState();

    Eigen::Vector3d cartAxis,cartAngles;

    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;
    Eigen::RowVectorXd operationalVelocity;

    Eigen::MatrixXd jacobian;
    Eigen::Affine3d bMee;

    double timeSinceStart;

    bool updated;
};

struct RobotMovementInfo
{
    RobotMovementInfo();
    ~RobotMovementInfo();

    Eigen::Affine3d target_xMe;
    Eigen::Affine3d motionReference; // prMx

    double acceleration;
    double velocity;
    double radius;
    double time;

    enum movementType
    {
        movej,
        movep,
        movel,
        speedj,
        speedl,
        stopj,
        stopl,
        undefinedMove
    };

    movementType typeOfMovement;

    Eigen::RowVectorXd targetJointVelocity;
};

#endif // ROBOTSTATE_H
