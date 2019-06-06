//
// Created by androst on 05.06.19.
//

#include <romocc/manipulators/ur5/Ur5KDLDefinition.h>
#include "catch.hpp"
#include "romocc/Robot.h"
#include <iostream>

namespace romocc {

TEST_CASE("Initialize robot with wrong address and check if connected", "[romocc][Robot]"){
    Robot::pointer robot = Robot::New();
    robot->configure(UR5, "dummy", 9999);
    robot->start();
    CHECK(robot->isConnected() == false);
}

TEST_CASE("Initialize robot and check coordinate system", "[romocc][Robot]") {
    Robot::pointer robot = Robot::New();
    CHECK(robot->getCoordinateSystem()->get_rMb().matrix() == Eigen::Affine3d::Identity().matrix());
    CHECK(robot->getCoordinateSystem()->get_eeMt().matrix() == Eigen::Affine3d::Identity().matrix());
}

TEST_CASE("Initialize robot and add update subscription", "[romocc][Robot]") {
    auto callback = [](){};

    Robot::pointer robot = Robot::New();
    robot->configure(UR5, "dummy", 9999);
    robot->start();
    robot->addUpdateSubscription(callback);

    void *subscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
    zmq_connect(subscriber, "tcp://localhost:5557");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    std::string msg_buffer;
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_recvmsg(subscriber, &message, 0);
    int size = zmq_msg_size(&message);
    msg_buffer.assign((const char *) zmq_msg_data(&message), size);
    std::cout << "Msg buffer : " << msg_buffer << std::endl;
    // TODO: Add check
}

TEST_CASE("Initialize robot and test kinematics", "[romocc][Robot]") {
    Robot::pointer robot = Robot::New();
    robot->configure(UR5, "localhost", 30003);

    KDL::ChainFkSolverPos_recursive fk_solver = robot->getCurrentState().getFKSolver();
    KDL::ChainIkSolverPos_NR ik_solver = robot->getCurrentState().getIKSolver();

    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=dh_home[i];}

    KDL::Frame target_pose = KDL::Frame::Identity();
    fk_solver.JntToCart(q_home, target_pose);

    for (unsigned int i = 0; i < 6; i++) {q_home(i)=q_home(i);}

    KDL::JntArray q_target = KDL::JntArray(6);
    ik_solver.CartToJnt(q_home, target_pose, q_target);

    CHECK(q_target.data == q_home.data);
}

}
