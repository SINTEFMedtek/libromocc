//
// Created by androst on 29.06.18.
//

#include <QApplication>
#include <iostream>

#include "Robot.h"
#include "manipulators/ur5/Ur5KDLDefinition.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    Robot ur5;
    ur5.configure(UR5, "localhost", 30003);
    ur5.start();

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

    //ur5.move(home, 0.3, 0.1);

    //Eigen::RowVectorXd q(6);
    //q << dh_home[0], dh_home[1], dh_home[2], dh_home[3], dh_home[4], dh_home[5];


    return app.exec();
}