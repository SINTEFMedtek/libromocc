//
// Created by androst on 29.06.18.
//

#include <QApplication>

#include "Robot.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    Robot ur5;
    ur5.setup(UR5, "localhost", 30003);
    ur5.initialize();

    //ur5.move(home, 0.3, 0.1);

    //Eigen::RowVectorXd q(6);
    //q << dh_home[0], dh_home[1], dh_home[2], dh_home[3], dh_home[4], dh_home[5];

    //KDL::JntArray q_home = KDL::JntArray(6);
    //for (unsigned int i = 0; i < 6; i++) {q_home(i)=dh_home[i];}

    return app.exec();
}