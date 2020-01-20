//
// Created by androst on 05.06.19.
//

#include "catch.hpp"
#include "romocc/Robot.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <romocc/manipulators/ur/UrKDLDefinition.h>

namespace romocc {

TEST_CASE("Initialize robot with wrong address and check if connected", "[romocc][Robot]"){
    Robot::pointer robot = Robot::New();
    robot->configure(Manipulator(), "dummy", 9999);
    robot->connect();
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
    robot->configure(Manipulator(), "dummy", 9999);
    robot->connect();
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
    robot->configure(Manipulator(), "192.168.140.116", 30003);

    std::shared_ptr<FKSolver> fk_solver = robot->getCurrentState()->getFKSolver();
    std::shared_ptr<IKSolver> ik_solver = robot->getCurrentState()->getIKSolver();

    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=Ur5::dh_home[i];}

    KDL::Frame target_pose = KDL::Frame::Identity();
    fk_solver->JntToCart(q_home, target_pose);

    for (unsigned int i = 0; i < 6; i++) {q_home(i)=q_home(i);}

    KDL::JntArray q_target = KDL::JntArray(6);
    ik_solver->CartToJnt(q_home, target_pose, q_target);

    CHECK(q_target.data == q_home.data);
}

TEST_CASE("Initialize robot and listen for new states", "[romocc][Robot]") {
    Robot::pointer robot = Robot::New();
    robot->configure(Manipulator(Manipulator::UR10), "192.168.153.131", 30003);
    robot->connect();

    for(int i = 0; i<100; i++)
    {
        std::cout << robot->getCurrentState()->getJointConfig().transpose() << std::endl;
    }
}

}
