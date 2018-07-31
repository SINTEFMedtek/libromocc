//
// Created by androst on 29.06.18.
//

#ifndef CORAH_ROBOT_H
#define CORAH_ROBOT_H

#include "utilities/corahHeaders.h"

#include "robotics/RobotState.h"
#include "communication/CommunicationInterface.h"

class Robot : public QObject
{
    Q_OBJECT

public:
    Robot();
    ~Robot();

    void configure(Manipulator manipulator, QString ip_address, int port);
    bool start();

    RobotState getCurrentState();

    void move(Eigen::Affine3d pose, double acc, double vel, double t=0, double rad=0);
    void move(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t=0, double rad=0);


private:
    void updateCurrentState(JointState state);

    CommunicationInterface mCommunicationInterface;
    RobotState mCurrentState;

};


#endif //CORAH_ROBOT_H
