//
// Created by androst on 29.06.18.
//

#include <algorithm>    // std::copy
#include <iostream>
#include "zmq.h"
#include <vector>
#include <sstream>
#include <cstring>
#include <iterator>
#include <thread>
#include <Eigen/Dense>

typedef unsigned char byte;

const int NUM_OF_JOINTS = 6;

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

double* arrayToLittleEndian(double* array)
{
    for(unsigned int i = 0; i < NUM_OF_JOINTS; i++)
        array[i] = toLittleEndian(&array[i]);
    return array;
}

struct __attribute__((packed)) ur_robot_state
{
    int32_t message_size_;
    double time_;
    double target_positions_[NUM_OF_JOINTS];
    double target_velocities_[NUM_OF_JOINTS];
    double target_accelerations_[NUM_OF_JOINTS];
    double target_currents_[NUM_OF_JOINTS];
    double target_torques_[NUM_OF_JOINTS];
    double actual_positions_[NUM_OF_JOINTS];
    double actual_velocities_[NUM_OF_JOINTS];
    double actual_currents_[NUM_OF_JOINTS];
    double joint_control_torques_[NUM_OF_JOINTS];
    double actual_tool_coordinates_[NUM_OF_JOINTS];
    double actual_tool_speed_[NUM_OF_JOINTS];
    double generalised_tool_forces_[NUM_OF_JOINTS];
    double target_tool_coordinates_[NUM_OF_JOINTS];
    double target_tool_speed_[NUM_OF_JOINTS];
    int64_t digital_pins_;
    double motor_temperatures_[NUM_OF_JOINTS];
    double controller_timer_;
    double test_value_;
    int64_t robot_mode_;
    int64_t joint_modes_[NUM_OF_JOINTS];
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
    double actual_joint_voltages_[NUM_OF_JOINTS];
};

typedef Eigen::Matrix<double,1,6> RowVector6d;

struct JointState
{
    RowVector6d jointConfiguration;
    RowVector6d jointVelocity;
    double timestamp = 0;
};


struct robot_state
{
    ur_robot_state* state_;
    JointState jointState;

    void update(void* buffer)
    {
        state_ = reinterpret_cast<ur_robot_state*>(buffer);

        double timestamp = toLittleEndian(&state_->time_);
        double* jointConfiguration = arrayToLittleEndian(state_->actual_positions_);
        double* jointVelocity = arrayToLittleEndian(state_->actual_velocities_);

        jointState.timestamp = timestamp;
        jointState.jointConfiguration = RowVector6d(jointConfiguration);
        jointState.jointVelocity = RowVector6d(jointVelocity);
    }
};



int getMessageSize(unsigned char* buffer)
{
    std::stringstream ss;
    unsigned int x;
    for(int i=0; i<sizeof(int); i++)
        ss << std::hex << (int)buffer[i];
    ss >> x;

    return static_cast<int>(x);
}

class streamer_task{
    public:
        robot_state* get_current_state(){ return rstate;};

        streamer_task(void* context, void* streamer, ur_robot_state* state)
        {
            ctx_ = context;
            streamer_ = streamer;
            state_ = state;
            rstate = new robot_state;
        };

        void start()
        {
            int rc = zmq_connect(streamer_, "tcp://localhost:30003");

            byte buffer[1044];

            try{
                while(true){
                    zmq_recv(streamer_, buffer, 1044, 0);
                    int packetLength = getMessageSize(buffer);

                    if(packetLength == 1044)
                    {
                        rstate->update(buffer);
                    }
                }
            }
            catch (std::exception &error){}
        }

    private:
        void* ctx_;
        void* streamer_;
        ur_robot_state* state_;
        robot_state* rstate;

};

int main(int argc, char *argv[])
{
    auto context = zmq_ctx_new();
    auto streamer = zmq_socket(context, ZMQ_STREAM);

    ur_robot_state *robot_state;

    streamer_task st(context, streamer, robot_state);
    std::thread thread_(std::bind(&streamer_task::start, &st));

    // Send motion command **
    int rc = zmq_connect(streamer, "tcp://localhost:30003");
    byte buffer[1044];
    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);
    char msg [] = "movel(p[-0.020114,-0.431763,0.288153,-0.001221,3.116276,0.038892], a=0.3, v=0.05, r=0)\n";
    zmq_send(streamer, id, id_size, ZMQ_SNDMORE);
    zmq_send(streamer, msg, strlen(msg), 0);
    // **

    while(true)
    {
        std::cout << st.get_current_state()->jointState.jointConfiguration << std::endl;
        std::cout << st.get_current_state()->jointState.jointVelocity << std::endl;
    }
}