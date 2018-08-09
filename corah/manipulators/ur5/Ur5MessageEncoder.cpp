#include "Ur5MessageEncoder.h"
#include <iostream>

QString Ur5MessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc,
                                      double vel, double t, double rad)
{
    acc = acc/1000; // mm -> m
    vel = vel/1000;
    rad = rad/1000;

    if(typeOfMovement==MotionType::movej)
        return movej(targetConfiguration,acc,vel,t,rad);
    else if(typeOfMovement==MotionType::movep)
        return movep(targetConfiguration,acc,vel,t,rad);
    else if(typeOfMovement==MotionType::speedl)
        return speedl(targetConfiguration,acc,t);
    else if(typeOfMovement == MotionType::speedj)
        return speedj(targetConfiguration,acc,t);
    else if(typeOfMovement == MotionType::stopj)
        return stopj(acc);
}

QString Ur5MessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::Affine3d pose, double acc, double vel, double t, double radius)
{
    pose.translation() = pose.translation()/1000;
    Eigen::RowVectorXd vector = AffineToRowVector(pose);
    return moveCommand(MotionType::movep, vector, acc, vel, t, radius);
}

QString Ur5MessageEncoder::stopCommand(MotionType typeOfStop, double acc)
{
    if(typeOfStop==MotionType::stopj)
        return stopj(acc);
    else if(typeOfStop==MotionType::stopl)
        return stopl(acc);
}

QString Ur5MessageEncoder::shutdownCommand()
{
    return powerdown();
}

QString Ur5MessageEncoder::movej(Eigen::RowVectorXd q,double a, double v,double t,double r)
{
    return QString("movej([%1,%2,%3,%4,%5,%6],a=%7,v=%8,t=%9,r=%10)")
            .arg(q(0)).arg(q(1)).arg(q(2)).arg(q(3)).arg(q(4)).arg(q(5)).arg(a).arg(v).arg(t).arg(r);
}

QString Ur5MessageEncoder::movep(Eigen::RowVectorXd op,double a, double v,double t,double r)
{
    return QString("movej(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8,r=%9)")
            .arg(op(0)).arg(op(1)).arg(op(2))
            .arg(op(3)).arg(op(4)).arg(op(5)).arg(a).arg(v).arg(r);
}

QString Ur5MessageEncoder::speedj(Eigen::RowVectorXd jointVelocity, double a, double t)
{
    return QString("speedj([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(jointVelocity(0)).arg(jointVelocity(1)).arg(jointVelocity(2)).arg(jointVelocity(3))
            .arg(jointVelocity(4)).arg(jointVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::speedl(Eigen::RowVectorXd operationalVelocity, double a, double t)
{
    return QString("speedl([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(operationalVelocity(0)).arg(operationalVelocity(1)).arg(operationalVelocity(2))
            .arg(operationalVelocity(3)).arg(operationalVelocity(4)).arg(operationalVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::stopl(double a)
{
    return QString("stopl(%1)").arg(a);
}

QString Ur5MessageEncoder::stopj(double a)
{
    return QString("stopj(%1)").arg(a);
}

QString Ur5MessageEncoder::powerdown()
{
    return QString("powerdown()");
}

QString Ur5MessageEncoder::textmsg(QString str)
{
    return QString("textmsg(%1)").arg(str);
}

QString Ur5MessageEncoder::sleep(double t)
{
    return QString("sleep(%1)").arg(t);
}