#ifndef ROMOCC_MESSAGEDECODER_H
#define ROMOCC_MESSAGEDECODER_H

#include "romocc/core/ForwardDeclarations.h"
#include "romocc/robotics/RobotState.h"

namespace romocc
{

struct ROMOCC_EXPORT JointState
{
    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;
    double timestamp = 0;
};

class ROMOCC_EXPORT MessageDecoder : public Object
{
    public:
        virtual JointState analyzeTCPSegment(unsigned char* packet) = 0;

};

}

#endif //ROMOCC_MESSAGEDECODER_H
