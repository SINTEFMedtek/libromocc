#ifndef ROMOCC_MESSAGEDECODER_H
#define ROMOCC_MESSAGEDECODER_H

#include "romocc/core/Object.h"
#include "romocc/core/ForwardDeclarations.h"

namespace romocc
{

struct ROMOCC_EXPORT ConfigState
{
    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;
    Eigen::RowVectorXd operationalConfiguration;
    Eigen::RowVectorXd operationalVelocity;
    Eigen::RowVectorXd operationalForce;
    int digitalOutputs;
    int digitalInputs;
    int configurableOutput;
    int configurableInputs;
    int toolOutputs;
    int toolInputs;
    int safetyMode;
    double timestamp = 0;
};

class ROMOCC_EXPORT MessageDecoder : public Object
{
    public:
        virtual ConfigState analyzeTCPSegment(unsigned char* packet) = 0;

};

}

#endif //ROMOCC_MESSAGEDECODER_H
