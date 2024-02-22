#include "UrMessageDecoder.h"
#include <iostream>
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
        double* operationalForce = romocc::arrayToLittleEndian(raw_state->generalised_tool_forces_);
        int allPOutputs = ntohd(raw_state->all_pouts_);  // TBD: make it a std::bitset<8>
        int allPInputs = ntohd(raw_state->all_pins_);
        int safetyMode = ntohd(raw_state->safety_mode_);

        int digitalOutputs = allPOutputs & 0xFF;
        int configurableOutputs = (allPOutputs >> 8) & 0xFF;
        int toolOutputs = (allPOutputs >> 16) & 0xFF;
        
        int digitalInputs = allPInputs & 0xFF;
        int configurableInputs = (allPInputs >> 8) & 0xFF;
        int toolInputs = (allPInputs >> 16) & 0xFF;

        TransformUtils::scaleTranslation(operationalConfiguration, 1000);
        TransformUtils::scaleTranslation(operationalVelocity, 1000);

        state.timestamp = timestamp;
        state.jointConfiguration = RowVector6d(jointConfiguration);
        state.jointVelocity = RowVector6d(jointVelocity);
        state.operationalConfiguration = RowVector6d(operationalConfiguration);
        state.operationalVelocity = RowVector6d(operationalVelocity);
        state.operationalForce = RowVector6d(operationalForce);
        state.digitalOutputs = digitalOutputs;
        state.digitalInputs = digitalInputs;
        state.configurableOutput = configurableOutputs;
        state.configurableInputs = configurableInputs;
        state.toolOutputs = toolOutputs;
        state.toolInputs = toolInputs;
        state.safetyMode = safetyMode;
    }
    return state;
}

}