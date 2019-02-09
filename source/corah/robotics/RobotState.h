#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "corah/utilities/corahHeaders.h"

namespace corah
{

/**
 * Struct that holds robot state information.
 *
 * \ingroup org_custusx_robotinterface
 *
 * \author Andreas Østvik
 * \date 2015-07-10
 */


class CORAH_EXPORT RobotState {

    public:

        RobotState();
        ~RobotState();

        void set_kdlchain(Manipulator manipulator);

        void set_jointState(RowVector6d q, RowVector6d q_vel, double timestamp);

        Transform3d getTransformToJoint(int jointNr = -1);

        Matrix6d getJacobian(int jointNr = -1);

        Vector6d jointConfiguration, jointVelocity, operationalConfiguration;

        Transform3d bMee;

        KDL::ChainFkSolverPos_recursive getFKSolver();

        KDL::ChainIkSolverPos_NR getIKSolver();

        double timestamp;

        Vector6d getOperationalVelocity();

    private:
        KDL::Chain mKDLChain;
        KDL::ChainFkSolverPos_recursive *mFKSolver;
        KDL::ChainIkSolverPos_NR *mIKSolver;
        KDL::ChainIkSolverVel_pinv *mIKSolverVel;
        KDL::ChainJntToJacSolver *mJacSolver;

        Transform3d transform_to_joint(RowVector6d jointConfig, int jointNr = -1);

};

}

#endif // ROBOTSTATE_H
