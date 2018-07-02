//
// Created by androst on 29.06.18.
//

#ifndef CORAH_ROBOT_H
#define CORAH_ROBOT_H

#include <QObject>

#include "robotics/RobotState.h"
#include "communication/CommunicationInterface.h"

class Robot : public QObject
{
    Q_OBJECT

public:
    Robot();
    ~Robot();

    void setup(Manipulator manipulator, QString ip_address, int port);
    bool initialize();

    RobotState getCurrentState();

    void move(Eigen::RowVectorXd jointConfiguration, double acc, double vel, double t=0, double rad=0);


private:
    void updateCurrentState(RobotState state);

    CommunicationInterface mCommunicationInterface;
    RobotState mCurrentState;

};


#endif //CORAH_ROBOT_H
