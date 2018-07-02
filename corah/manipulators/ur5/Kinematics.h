#ifndef UR5KINEMATICS_H
#define UR5KINEMATICS_H

#include "org_custusx_robot_ur5_Export.h"
#include "cxVector3D.h"
#include "cxTransform3D.h"



namespace cx
{
/**
 * Implementation of UR5 Kinematics.
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 */

class org_custusx_robot_ur5_EXPORT Ur5Kinematics
{
public:    
    static Transform3D forward(Eigen::RowVectorXd jointConfiguration);
    static Eigen::MatrixXd jacobian(Eigen::RowVectorXd jointConfiguration);
    static Eigen::MatrixXd jacobian2(Eigen::RowVectorXd jointConfiguration);
    static Eigen::MatrixXd invJacobian(Eigen::RowVectorXd jointConfiguration);

    Eigen::RowVectorXd inverseJmethod(Transform3D desiredPose, Eigen::RowVectorXd guessedJointConfiguration);

    static Transform3D T01(Eigen::RowVectorXd jointConfiguration);
    static Transform3D T02(Eigen::RowVectorXd jointConfiguration);
    static Transform3D T03(Eigen::RowVectorXd jointConfiguration);
    static Transform3D T04(Eigen::RowVectorXd jointConfiguration);
    static Transform3D T05(Eigen::RowVectorXd jointConfiguration);

    static Vector3D T2transl(Transform3D T);
    static Vector3D T2rangles(Transform3D T);
    static Eigen::RowVectorXd T2OperationalConfiguration(Transform3D pose);



private:
    Transform3D poseToMatrix(Eigen::RowVectorXd poseConfiguration);

    Eigen::MatrixXd getRotation(Transform3D pose) const;
    Eigen::RowVectorXd errorVector(Transform3D desiredPose, Transform3D currentPose) const;
    Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd matrix) const;

    static const double d1(){return 0.089159;}
    static const double a2(){return -0.42500;}
    static const double a3(){return -0.39225;}
    static const double d4(){return 0.10915;}
    static const double d5(){return 0.09465;}
    static const double d6(){return 0.0823;}

};

} // cx

#endif // UR5KINEMATICS_H