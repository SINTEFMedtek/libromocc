#include "UrMessageDecoder.h"
#include "romocc/utilities/ZMQUtils.h"

#include <set>
#include <iostream>

namespace romocc
{

JointState UrMessageDecoder::analyzeTCPSegment(unsigned char* packet)
{
    JointState state;
    if(romocc::packageSize(packet)>=764 ||  romocc::packageSize(packet)<=1116)
    {
        raw_ur_state* raw_state = reinterpret_cast<raw_ur_state*>(packet);
        double timestamp = ntohd(raw_state->time_);
        double* jointConfiguration = romocc::arrayToLittleEndian(raw_state->actual_positions_);
        double* jointVelocity = romocc::arrayToLittleEndian(raw_state->actual_velocities_);

        state.timestamp = timestamp;
        state.jointConfiguration = RowVector6d(jointConfiguration);
        state.jointVelocity = RowVector6d(jointVelocity);
    }
    return state;
}

}