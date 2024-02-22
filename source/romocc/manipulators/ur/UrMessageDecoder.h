#ifndef URRECEIVE_H
#define URRECEIVE_H

#include <romocc/core/SmartPointers.h>
#include "romocc/robotics/RobotState.h"
#include "romocc/communication/MessageDecoder.h"

namespace romocc
{

/**
 * Class that handles UR robot recieved messages.
 *
 * \author Andreas Østvik
 *
 */


class ROMOCC_EXPORT UrMessageDecoder : public MessageDecoder
{
    ROMOCC_OBJECT(UrMessageDecoder)

    public:
        ConfigState analyzeTCPSegment(unsigned char* packet) override;

    private:
        static const int NUMBER_OF_JOINTS = 6;

        PACK(struct raw_ur_state
        {
                int32_t message_size_;
                uint64_t time_;
                double target_positions_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_velocities_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_accelerations_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_currents_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_torques_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double actual_positions_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double actual_velocities_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double actual_currents_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double joint_control_torques_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double actual_tool_coordinates_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double actual_tool_speed_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double generalised_tool_forces_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_tool_coordinates_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double target_tool_speed_[UrMessageDecoder::NUMBER_OF_JOINTS];
                int64_t all_pins_;
                double motor_temperatures_[UrMessageDecoder::NUMBER_OF_JOINTS];
                double controller_timer_;
                double test_value_;
                int64_t robot_mode_;
                int64_t joint_modes_[UrMessageDecoder::NUMBER_OF_JOINTS];
                int64_t safety_mode_;
                int64_t unused_1_[6];
                double tool_accelerometer_values_[3];
                int64_t unused_2_[6];
                double speed_scaling_;
                double linear_momentum_norm_;
                int64_t unused_3_[2];
                double masterboard_main_voltage_;
                double masterboard_robot_voltage_;
                double masterboard_robot_current_;
                double actual_joint_voltages_[UrMessageDecoder::NUMBER_OF_JOINTS];
                int64_t all_pouts_;
        });
};



}

#endif // URRECEIVE_H
