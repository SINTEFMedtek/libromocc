#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include <utilities/corahHeaders.h>

/**
 * Struct that holds robot motion information.
 *
 * \ingroup org_custusx_robotics
 *
 * \author Andreas Østvik
 * \date 2015-07-10
 */

struct RobotMotion
{
    Transform3d targetPose;
    RowVector6d targetJointConfiguration;

    double acceleration;
    double velocity;
    double time = 0;
    double blendRadius = 0;

    MotionType motionType;
};

class RobotMotionQueue
{
    public:
        void add(Transform3d target, MotionType type, double acc, double vel, double time = 0, double blendRadius=0)
        {
            RobotMotion motion;
            motion.targetPose = target;
            motion.motionType = type;
            motion.acceleration = acc;
            motion.velocity = vel;
            motion.time = time;
            motion.blendRadius = blendRadius;

            mQueue.push_back(motion);
        };

        void clear(){mQueue.clear();};

        MotionQueue getQueue(){ return mQueue;};

    private:
        MotionQueue mQueue;

};


#endif //ROBOTMOTION_H
