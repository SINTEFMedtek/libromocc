//
// Created by androst on 29.06.18.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "corah/Robot.h"
#include "corah/manipulators/ur5/Ur5KDLDefinition.h"

using namespace corah;

int main(int argc, char *argv[])
{
    Robot ur5;
    ur5.configure(UR5, "localhost", 30003);
    ur5.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Finished waiting" << std::endl;
    std::cout << ur5.getCurrentState().jointConfiguration << std::endl;

    KDL::ChainFkSolverPos_recursive fk_solver = ur5.getCurrentState().getFKSolver();
    KDL::ChainIkSolverPos_NR ik_solver = ur5.getCurrentState().getIKSolver();

    KDL::JntArray q_home = KDL::JntArray(6);
    for (unsigned int i = 0; i < 6; i++) {q_home(i)=dh_home[i];}

    KDL::Frame target_pose = KDL::Frame::Identity();
    fk_solver.JntToCart(q_home, target_pose);

    for (unsigned int i = 0; i < 6; i++) {q_home(i)=q_home(i)+0.05;}

    KDL::JntArray q_target = KDL::JntArray(6);
    std::cout << ik_solver.CartToJnt(q_home, target_pose, q_target) << std::endl;
    std::cout << KDLFrameToEigenAffine(target_pose).matrix() << std::endl;
    std::cout << q_target.data << std::endl;

    ur5.move(corah::MotionType::movej, q_target.data, 50, 25);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << ur5.getCurrentState().jointConfiguration << std::endl;
}