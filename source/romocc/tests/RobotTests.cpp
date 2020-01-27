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
    robot->configure(Manipulator(Manipulator::UR10), "192.168.153.131", 30003);
    robot->connect();

    std::shared_ptr<FKSolver> fk_solver = robot->getCurrentState()->getFKSolver();
    std::shared_ptr<IKSolver> ik_solver = robot->getCurrentState()->getIKSolver();

    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=Ur10::dh_home[i];}

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

TEST_CASE("Kinematics test", "[romocc][Robot]"){
    romocc::Vector6d current;
    current << -424, -564., 161., 0, 0.96*M_PI, 0.723*M_PI;
    //std::cout << current.transpose() << "\n" << std::endl;

    auto affine = TransformUtils::Affine::toAffine3DFromVector6D(current);
    //std::cout << affine.matrix() << "\n" << std::endl;

    auto robot = Robot::New();
    robot->configure(Manipulator(Manipulator::UR10), "192.168.153.131", 30003);
    robot->connect();

    std::shared_ptr<FKSolver> fk_solver = robot->getCurrentState()->getFKSolver();
    std::shared_ptr<IKSolver> ik_solver = robot->getCurrentState()->getIKSolver();

    auto scaled_affine = TransformUtils::Affine::scaleTranslation(affine, 1/1000.0);
    auto target_joints = robot->getCurrentState()->operationalConfigToJointConfig(scaled_affine);
    std::cout << target_joints.transpose() << "\n" << std::endl;

    auto target_pose = TransformUtils::kdl::fromAffine(scaled_affine);
    auto q_current = TransformUtils::kdl::fromVector6D(current);

    double home[6] = {-2.37, -1.67, -1.64 ,-1.29, 0.80, 0.35};
    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=home[i];}

    //std::cout << q_home(0) << " " << q_home(1) << " " << q_home(2) << " " << q_home(3) << " " << q_home(4) << " " << q_home(5) << std::endl;
    //std::cout << target_pose.M(0,0) << " " << target_pose.M(0,1) << " " << target_pose.M(0,2) << "\n" << std::endl;
    //std::cout << target_pose.p[0] << " " << target_pose.p[1] << " " << target_pose.p[2] << "\n"<< std::endl;

    KDL::JntArray q_target = KDL::JntArray(6);
    int status = ik_solver->CartToJnt(q_home, target_pose, q_target);
    std::cout << status << std::endl;
    std::cout << q_target.data.transpose() << std::endl;
}

}
