#include "corah/utilities/corahHeaders.h"
#include "corah/robotics/RobotState.h"

#ifndef CORAH_MESSAGEDECODER_H
#define CORAH_MESSAGEDECODER_H

namespace corah
{

struct CORAH_EXPORT JointState
{
    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;
    double timestamp = 0;
};

class CORAH_EXPORT MessageDecoder {

    public:
        virtual JointState analyzeTCPSegment(unsigned char* packet) = 0;

    protected:
        int packageSize(unsigned char* package);

        template<typename T> static T toLittleEndian(T *big_endian_number)
        {
            T little_endian_number;
            char *big_endian_data    = reinterpret_cast<char*>(big_endian_number);
            char *little_endian_data = reinterpret_cast<char*>(&little_endian_number);

            size_t length = sizeof(T);
            for (size_t i = 0; i < length; ++i)
            {
                little_endian_data[i] = big_endian_data[length - i - 1];
            }
            return little_endian_number;
        }

        double* arrayToLittleEndian(double* array, unsigned int N = 6);
};

}

#endif //CORAH_MESSAGEDECODER_H
