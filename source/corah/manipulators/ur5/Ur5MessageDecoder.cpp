#include "Ur5MessageDecoder.h"
#include <set>
#include <iostream>

namespace corah
{

JointState Ur5MessageDecoder::analyzeTCPSegment(unsigned char* packet)
{
    JointState state;
    if(packageSize(packet)==812 || packageSize(packet) == 1044)
    {
        raw_ur5_state* raw_state = reinterpret_cast<raw_ur5_state*>(packet);

        double timestamp = toLittleEndian(&raw_state->time_);
        double* jointConfiguration = arrayToLittleEndian(raw_state->actual_positions_);
        double* jointVelocity = arrayToLittleEndian(raw_state->actual_velocities_);

        state.timestamp = timestamp;
        state.jointConfiguration = RowVector6d(jointConfiguration);
        state.jointVelocity = RowVector6d(jointVelocity);
    }
    return state;
}

}