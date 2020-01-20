#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <mutex>
#include "romocc/core/Object.h"
#include "romocc/utilities/MathUtils.h"

namespace romocc
{

class MessageDecoder;

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
        ~RobotState(){};

        void setManipulator(Manipulator manipulator);
        void unpack(uint8_t* buffer);

        Transform3d getTransformToJoint(int jointNr = -1);
        Transform3d get_bMee();

        double getTimestamp();
        Matrix6d getJacobian(int jointNr = -1);
        Vector6d getJointConfig();
        Vector6d getJointVelocity();
        Vector6d getOperationalConfig();
        Vector6d getOperationalVelocity();
        Vector6d operationalConfigToJointConfig(Transform3d transform);

        std::shared_ptr<FKSolver> getFKSolver();
        std::shared_ptr<IKSolver> getIKSolver();

    private:
        SharedPointer<MessageDecoder> mDecoder;
        void setKDLchain(Manipulator manipulator);
        void setDecoder(Manipulator manipulator);
        void setState(RowVector6d q, RowVector6d q_vel, double timestamp);

        double mTimestamp;
        Vector6d mJointConfiguration;
        Vector6d mJointVelocity;
        Vector6d mOperationalConfiguration;
        Transform3d m_bMee;

        std::mutex mValueLock;

        RobotChain mKDLChain;
        std::shared_ptr<FKSolver> mFKSolver;
        std::shared_ptr<IKSolver> mIKSolver;
        std::shared_ptr<IKVelSolver> mIKSolverVel;
        std::shared_ptr<JacobianSolver> mJacSolver;

        Transform3d transform_to_joint(RowVector6d jointConfig, int jointNr = -1);
};

}

#endif // ROBOTSTATE_H
