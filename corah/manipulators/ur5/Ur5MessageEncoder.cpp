#include "Ur5MessageEncoder.h"

QString Ur5MessageEncoder::moveJoints(Eigen::RowVectorXd jointConfig,double acc, double vel,double t,double r)
{
    return movej(jointConfig, acc, vel, t, r);
}

QString Ur5MessageEncoder::movePose(Eigen::Affine3d pose, double acc, double vel, double t, double radius)
{
    Eigen::RowVectorXd vec = AffineToRowVector(pose);
    return movep(vec, acc, vel, t, radius);
}

QString Ur5MessageEncoder::movej(Eigen::RowVectorXd q,double a, double v,double t,double r)
{
    return QString("movej([%1,%2,%3,%4,%5,%6],a=%7,v=%8,t=%9,r=%10)")
            .arg(q(0)).arg(q(1)).arg(q(2)).arg(q(3)).arg(q(4)).arg(q(5)).arg(a).arg(v).arg(t).arg(r);
}

QString Ur5MessageEncoder::movep(Eigen::RowVectorXd op,double a, double v,double t,double r)
{
    return QString("movej(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8,r=%9)")
            .arg(op(0)/1000).arg(op(1)/1000).arg(op(2)/1000)
            .arg(op(3)).arg(op(4)).arg(op(5)).arg(a).arg(v).arg(r);
}

QString Ur5MessageEncoder::speedj(Eigen::RowVectorXd jointVelocity, double a, double t)
{
    return QString("speedj([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(jointVelocity(0)).arg(jointVelocity(1)).arg(jointVelocity(2)).arg(jointVelocity(3))
            .arg(jointVelocity(4)).arg(jointVelocity(5)).arg(a).arg(t);
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