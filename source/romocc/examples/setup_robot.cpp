//
// Created by androst on 29.06.18.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "romocc/Robot.h"
#include "romocc/manipulators/ur5/Ur5KDLDefinition.h"

using namespace romocc;

int main(int argc, char *argv[])
{
    Robot ur5;
    ur5.configure(UR5, "localhost", 30003);
    ur5.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Finished waiting" << std::endl;
    std::cout << ur5.getCurrentState().jointConfiguration << std::endl;
    auto initJointConfig = ur5.getCurrentState().jointConfiguration;

    KDL::ChainFkSolverPos_recursive fk_solver = ur5.getCurrentState().getFKSolver();
    KDL::ChainIkSolverPos_NR ik_solver = ur5.getCurrentState().getIKSolver();

    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=dh_home[i];}

    KDL::Frame target_pose = KDL::Frame::Identity();
    fk_solver.JntToCart(q_home, target_pose);

    for (unsigned int i = 0; i < 6; i++) {q_home(i)=q_home(i)+0.05;}

    KDL::JntArray q_target = KDL::JntArray(6);
    std::cout << ik_solver.CartToJnt(q_home, target_pose, q_target) << std::endl;
    std::cout << TransformUtils::kdl::toAffine(target_pose).matrix() << std::endl;
    std::cout << q_target.data << std::endl;

    ur5.move(romocc::MotionType::movej, q_target.data, 50, 25);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    void *ctx = zmq_ctx_new();
    void *subscriber = zmq_socket(ctx, ZMQ_SUB);
    std::string msg_buffer;
    zmq_connect(subscriber, "tcp://localhost:5557");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_recvmsg(subscriber, &message, 0);
    int size = zmq_msg_size(&message);
    msg_buffer.assign((const char *) zmq_msg_data(&message), size);
    std::cout << "Msg buffer : " << msg_buffer << std::endl;

    std::cout << ur5.getCurrentState().jointConfiguration << std::endl;
    ur5.move(romocc::MotionType::movej, initJointConfig, 50, 25);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << ur5.getCurrentState().jointConfiguration << std::endl;

}