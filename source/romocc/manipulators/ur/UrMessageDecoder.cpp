#include "UrMessageDecoder.h"

#include "romocc/utilities/MathUtils.h"
#include "romocc/utilities/ZMQUtils.h"


namespace romocc
{

ConfigState UrMessageDecoder::analyzeTCPSegment(unsigned char* packet)
{
    ConfigState state;
    if(romocc::packageSize(packet)>=764 ||  romocc::packageSize(packet)<=1116)
    {
        raw_ur_state* raw_state = reinterpret_cast<raw_ur_state*>(packet);
        double timestamp = ntohd(raw_state->time_);
        double* jointConfiguration = romocc::arrayToLittleEndian(raw_state->actual_positions_);
        double* jointVelocity = romocc::arrayToLittleEndian(raw_state->actual_velocities_);
        double* operationalConfiguration = romocc::arrayToLittleEndian(raw_state->actual_tool_coordinates_);
        double* operationalVelocity = romocc::arrayToLittleEndian(raw_state->actual_tool_speed_);

        TransformUtils::scaleTranslation(operationalConfiguration, 1000);
        TransformUtils::scaleTranslation(operationalVelocity, 1000);

        state.timestamp = timestamp;
        state.jointConfiguration = RowVector6d(jointConfiguration);
        state.jointVelocity = RowVector6d(jointVelocity);
        state.operationalConfiguration = RowVector6d(operationalConfiguration);
        state.operationalVelocity = RowVector6d(operationalVelocity);
    }
    return state;
}

}