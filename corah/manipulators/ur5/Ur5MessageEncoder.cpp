#include "Ur5MessageEncoder.h"

QString Ur5MessageEncoder::moveJoints(Eigen::RowVectorXd jointConfig,double acc, double vel,double t,double r)
{
    return movej(jointConfig, acc, vel, t, r);
}


QString Ur5MessageEncoder::movej(Eigen::RowVectorXd p,double a, double v,double t,double r)
{
    return QString("movej([%1,%2,%3,%4,%5,%6],a=%7,v=%8,t=%9,r=%10)")
            .arg(p(0)).arg(p(1)).arg(p(2)).arg(p(3)).arg(p(4)).arg(p(5)).arg(a).arg(v).arg(t).arg(r);
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