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


class ROMOCC_EXPORT RobotState {
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

        KDL::ChainFkSolverPos_recursive getFKSolver();
        KDL::ChainIkSolverPos_NR getIKSolver();

    private:
        double mTimestamp;
        Vector6d mJointConfiguration, mJointVelocity, mOperationalConfiguration;
        Transform3d m_bMee;

        KDL::Chain mKDLChain;
        KDL::ChainFkSolverPos_recursive *mFKSolver;
        KDL::ChainIkSolverPos_NR *mIKSolver;
        KDL::ChainIkSolverVel_pinv *mIKSolverVel;
        KDL::ChainJntToJacSolver *mJacSolver;

        Transform3d transform_to_joint(RowVector6d jointConfig, int jointNr = -1);

        std::weak_ptr<RobotState> mPtr;
};

}

#endif // ROBOTSTATE_H
