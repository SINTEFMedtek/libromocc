#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "romocc/core/Object.h"
#include "romocc/utilities/MathUtils.h"

namespace romocc
{

/**
 * Struct that holds robot state information.
 *
 * \author Andreas Østvik
 */


class ROMOCC_EXPORT RobotState : public Object
{
    ROMOCC_OBJECT(RobotState)

    public:
        RobotState();
        ~RobotState();

        void setKDLchain(Manipulator manipulator);
        void setState(RowVector6d q, RowVector6d q_vel, double timestamp);

        Transform3d getTransformToJoint(int jointNr = -1) const;
        Transform3d get_bMee() const;

        Matrix6d getJacobian(int jointNr = -1) const;
        Vector6d getJointConfig() const;
        Vector6d getJointVelocity() const;
        Vector6d getOperationalConfig() const;
        Vector6d getOperationalVelocity() const;

        std::shared_ptr<FKSolver> getFKSolver();
        std::shared_ptr<IKSolver> getIKSolver();

    private:
        double mTimestamp;
        Vector6d mJointConfiguration, mJointVelocity, mOperationalConfiguration;
        Transform3d m_bMee;

        RobotChain mKDLChain;
        std::shared_ptr<FKSolver> mFKSolver;
        std::shared_ptr<IKSolver> mIKSolver;
        std::shared_ptr<IKVelSolver> mIKSolverVel;
        std::shared_ptr<JacobianSolver> mJacSolver;

        Transform3d transform_to_joint(RowVector6d jointConfig, int jointNr = -1);
};

}

#endif // ROBOTSTATE_H
