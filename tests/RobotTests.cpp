//
// Created by androst on 05.06.19.
//

#include "catch/catch.hpp"
#include "romocc/Robot.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <romocc/manipulators/ur/UrKDLDefinition.h>



namespace romocc {

TEST_CASE("Initialize robot with wrong address and check if connected", "[romocc][Robot]"){
    auto robot = Robot::New();
    robot->configure(Manipulator(), "dummy", 9999);
    robot->connect();
    CHECK(robot->isConnected() == false);
}

TEST_CASE("Initialize robot and check coordinate system", "[romocc][Robot]") {
    auto robot = Robot::New();
    CHECK(robot->getCoordinateSystem()->get_rMb().matrix() == Eigen::Affine3d::Identity().matrix());
    CHECK(robot->getCoordinateSystem()->get_eeMt().matrix() == Eigen::Affine3d::Identity().matrix());
}

TEST_CASE("Initialize robot and test kinematics", "[romocc][Robot]") {
    Robot::pointer robot = Robot::New();
    robot->configure(Manipulator(ManipulatorType::UR10), "192.168.153.131", 30003);
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
    robot->configure(Manipulator(ManipulatorType::UR10), "192.168.153.131", 30003);
    robot->connect();

    for(int i = 0; i<100; i++)
    {
        std::cout << robot->getCurrentState()->getJointConfig().transpose() << std::endl;
    }
}

TEST_CASE("Kinematics test", "[romocc][Robot]"){
    romocc::Vector6d current;
    current << -424, -564., 161., 0, 0.96*M_PI, 0.723*M_PI;

    auto affine = TransformUtils::Affine::toAffine3DFromVector6D(current);

    auto robot = Robot::New();
    robot->configure(Manipulator(ManipulatorType::UR10), "192.168.153.131", 30003);
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

    KDL::JntArray q_target = KDL::JntArray(6);
    int status = ik_solver->CartToJnt(q_home, target_pose, q_target);
    std::cout << status << std::endl;
    std::cout << q_target.data.transpose() << std::endl;
}

}
