#ifndef UR5RECEIVE_H
#define UR5RECEIVE_H

#include <romocc/core/SmartPointers.h>
#include "romocc/robotics/RobotState.h"
#include "romocc/communication/MessageDecoder.h"

namespace romocc
{

/**
 * Class that handles UR5 robot recieved messages.
 *
 * \author Andreas Østvik
 *
 */


class ROMOCC_EXPORT Ur5MessageDecoder : public MessageDecoder
{
    ROMOCC_OBJECT(Ur5MessageDecoder)

    public:
        virtual JointState analyzeTCPSegment(unsigned char* packet);

    private:
        static const int NUMBER_OF_JOINTS = 6;

        struct __attribute__((packed)) raw_ur5_state
        {
            int32_t message_size_;
            double time_;
            double target_positions_[NUMBER_OF_JOINTS];
            double target_velocities_[NUMBER_OF_JOINTS];
            double target_accelerations_[NUMBER_OF_JOINTS];
            double target_currents_[NUMBER_OF_JOINTS];
            double target_torques_[NUMBER_OF_JOINTS];
            double actual_positions_[NUMBER_OF_JOINTS];
            double actual_velocities_[NUMBER_OF_JOINTS];
            double actual_currents_[NUMBER_OF_JOINTS];
            double joint_control_torques_[NUMBER_OF_JOINTS];
            double actual_tool_coordinates_[NUMBER_OF_JOINTS];
            double actual_tool_speed_[NUMBER_OF_JOINTS];
            double generalised_tool_forces_[NUMBER_OF_JOINTS];
            double target_tool_coordinates_[NUMBER_OF_JOINTS];
            double target_tool_speed_[NUMBER_OF_JOINTS];
            int64_t digital_pins_;
            double motor_temperatures_[NUMBER_OF_JOINTS];
            double controller_timer_;
            double test_value_;
            int64_t robot_mode_;
            int64_t joint_modes_[NUMBER_OF_JOINTS];
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
            double actual_joint_voltages_[NUMBER_OF_JOINTS];
        };
};



}

#endif // UR5RECEIVE_H
