#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H
#define _USE_MATH_DEFINES

#include <mutex>
#include <math.h>

#include "romocc/core/Object.h"
#include "romocc/utilities/MathUtils.h"
#include "romocc/communication/MessageDecoder.h"

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
        Vector6d getOperationalForce();
        Vector6d operationalConfigToJointConfig(Transform3d transform);
        Transform3d jointConfigToOperationalConfig(Vector6d jointConfig);

        std::shared_ptr<FKSolver> getFKSolver();
        std::shared_ptr<IKSolver> getIKSolver();

    private:
        std::shared_ptr<MessageDecoder> mDecoder;
        void setKDLchain(Manipulator manipulator);
        void setDecoder(Manipulator manipulator);
        void setState(romocc::ConfigState configState);

        double mTimestamp;
        Vector6d mJointConfiguration;
        Vector6d mJointVelocity;
        Vector6d mOperationalConfiguration;
        Vector6d mOperationalVelocity;
        Vector6d mOperationalForce;
        Transform3d m_bMee;
        Manipulator mManipulator;

        std::mutex mValueLock;

        RobotChain mKDLChain;
        std::shared_ptr<FKSolver> mFKSolver;
        std::shared_ptr<IKSolver> mIKSolver;
        std::shared_ptr<IKVelSolver> mIKSolverVel;
        std::shared_ptr<JacobianSolver> mJacSolver;

        double mJointMinimum[6] = {-2*M_PI, -2*M_PI, -2*M_PI , -2*M_PI, -2*M_PI, -2*M_PI};
        double mJointMaximum[6] = {2*M_PI, 2*M_PI, 2*M_PI , 2*M_PI, 2*M_PI, 2*M_PI};
        double mJointInit[6] = {M_PI_4-0.1, -M_PI_2+0.1, M_PI_2+0.1, -M_PI_2-0.1, M_PI_2+0.1, 0.1+M_PI/4};

        Transform3d transform_to_joint(RowVector6d jointConfig, int jointNr = -1);
};

}

#endif // ROBOTSTATE_H
