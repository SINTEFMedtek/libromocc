//
// Created by androst on 29.06.18.
//

#ifndef CORAH_ROBOT_H
#define CORAH_ROBOT_H

#include "utilities/corahHeaders.h"

#include "robotics/RobotState.h"
#include "communication/CommunicationInterface.h"

typedef std::shared_ptr<class Robot> RobotPtr;

class Robot : public QObject
{
    Q_OBJECT


public:
    Robot();
    ~Robot();

    void configure(Manipulator manipulator, QString ip_address, int port);
    bool start();
    bool isConnected();
    bool disconnect();
    void shutdown();

    RobotState getCurrentState();


    template <class Target>
    void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0);
    void stopMove(MotionType type, double acc);

    void set_eeMt(Eigen::Affine3d eeMt);
    void set_rMb(Eigen::Affine3d rMb);

    signals:
    void stateUpdated();

private:
    void updateCurrentState(JointState state);

    CommunicationInterface mCommunicationInterface;
    RobotState mCurrentState;

    Eigen::Affine3d eeMt, rMb;



};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad)
{
    mCommunicationInterface.move(type, target, acc, vel, t, rad);
};


#endif //CORAH_ROBOT_H
