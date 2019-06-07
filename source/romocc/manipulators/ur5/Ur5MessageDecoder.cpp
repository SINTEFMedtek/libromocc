#include "Ur5MessageDecoder.h"
#include "romocc/utilities/ZMQUtils.h"

#include <set>

namespace romocc
{

JointState Ur5MessageDecoder::analyzeTCPSegment(unsigned char* packet)
{
    JointState state;
    if(romocc::packageSize(packet)==812 || romocc::packageSize(packet) == 1044)
    {
        raw_ur5_state* raw_state = reinterpret_cast<raw_ur5_state*>(packet);

        double timestamp = toLittleEndian(&raw_state->time_);
        double* jointConfiguration = romocc::arrayToLittleEndian(raw_state->actual_positions_);
        double* jointVelocity = romocc::arrayToLittleEndian(raw_state->actual_velocities_);

        state.timestamp = timestamp;
        state.jointConfiguration = RowVector6d(jointConfiguration);
        state.jointVelocity = RowVector6d(jointVelocity);
    }
    return state;
}

}