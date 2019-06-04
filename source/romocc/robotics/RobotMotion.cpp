#include "RobotMotion.h"

namespace romocc
{

Vector6d RobotMotionUtils::calcJointVelocity(Transform3d targetPose, Transform3d currentPose, Matrix6d jacobian, double velocity)
{
    Vector3d tangent = (targetPose.translation()-currentPose.translation());
    tangent = tangent/tangent.norm();

    Vector3d opVelocity =  tangent*velocity/1000;

    // Todo: Estimate rx, ry, rz
    //Vector3d rtangent = AffineToAxisAngle(target.targetPose)-AffineToAxisAngle(mCurrentState.bMee*eeMt);
    //rtangent = rtangent/rtangent.norm();
    //Vector3d rvelocity = rvelocity*target.velocity/1000;

    Vector6d velocityEndEffector;
    velocityEndEffector << opVelocity(0), opVelocity(1), opVelocity(2), 0, 0, 0;

    Vector6d jointVelocity = jacobian.inverse()*velocityEndEffector;

    return jointVelocity;
}

}
