#include "MessageEncoder.h"
#include "robotics/RobotState.h"
#include "Kinematics.h"

namespace cx
{

QString Ur5MessageEncoder::movej(Ur5State p,double a, double v,double r)
{
    return QString("movej(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8,r=%9)")
            .arg(p.cartAxis(0)/1000).arg(p.cartAxis(1)/1000).arg(p.cartAxis(2)/1000)
            .arg(p.cartAngles(0)).arg(p.cartAngles(1)).arg(p.cartAngles(2)).arg(a/1000).arg(v/1000).arg(r/1000);
}

QString Ur5MessageEncoder::movej(Eigen::RowVectorXd p,double a, double v,double t,double r)
{
    return QString("movej([%1,%2,%3,%4,%5,%6],a=%7,v=%8,r=%9)")
            .arg(p(0)).arg(p(1)).arg(p(2)).arg(p(3)).arg(p(4)).arg(p(5)).arg(a).arg(v).arg(t).arg(r);
}

QString Ur5MessageEncoder::movej(Eigen::RowVectorXd p,double t)
{
    return QString("movej([%1,%2,%3,%4,%5,%6],a=1.4,v=1.05,t=%7)")
            .arg(p(0)).arg(p(1)).arg(p(2)).arg(p(3)).arg(p(4)).arg(p(5)).arg(t);
}

QString Ur5MessageEncoder::movejp(Eigen::RowVectorXd p,double a, double v,double t,double r)
{
    return QString("movej(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8,r=%9)")
            .arg(p(0)/1000).arg(p(1)/1000).arg(p(2)/1000)
            .arg(p(3)).arg(p(4)).arg(p(5)).arg(a).arg(v).arg(r);
}

QString Ur5MessageEncoder::movej(Ur5MovementInfo minfo)
{
    Eigen::RowVectorXd op = Ur5Kinematics::T2OperationalConfiguration(minfo.target_xMe);

    if(minfo.acceleration == 0 && minfo.velocity == 0 && minfo.time != 0)
    {
        return QString("movej(p[%1,%2,%3,%4,%5,%6],a=1.4,v=1.05,t=%7)")
                .arg(op(0)/1000).arg(op(1)/1000).arg(op(2)/1000).arg(op(3)).arg(op(4)).arg(op(5))
                .arg(minfo.time);
    }
    else
    {
        return QString("movej(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8)")
                .arg(op(0)/1000).arg(op(1)/1000).arg(op(2)/1000).arg(op(3)).arg(op(4)).arg(op(5))
                .arg(minfo.acceleration/1000).arg(minfo.velocity/1000);
    }
}

QString Ur5MessageEncoder::movel(Ur5State p,double a, double v)
{
    return QString("movel(p[%1,%2,%3,%4,%5,%6],a=%7,v=%8)")
            .arg(p.cartAxis(0)/1000).arg(p.cartAxis(1)/1000).arg(p.cartAxis(2)/1000)
            .arg(p.cartAngles(0)).arg(p.cartAngles(1)).arg(p.cartAngles(2)).arg(a).arg(v);
}

QString Ur5MessageEncoder::movec(Ur5State pose_via,Ur5State pose_to, double a, double v, double r)
{
    return QString("movec(p[%1,%2,%3,%4,%5,%6],p[%7,%8,%9,%10,%11,%12],a=%13,v=%14,r=%15")
            .arg(pose_via.cartAxis(0)/1000).arg(pose_via.cartAxis(1)/1000).arg(pose_via.cartAxis(2)/1000)
            .arg(pose_via.cartAngles(0)).arg(pose_via.cartAngles(1)).arg(pose_via.cartAngles(2))
            .arg(pose_to.cartAxis(0)/1000).arg(pose_to.cartAxis(1)/1000).arg(pose_to.cartAxis(2)/1000)
            .arg(pose_to.cartAngles(0)).arg(pose_to.cartAngles(1)).arg(pose_to.cartAngles(2))
            .arg(a).arg(v).arg(r);
}

QString Ur5MessageEncoder::speedj(Ur5State p, double a, double t)
{
    return QString("speedj([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(p.jointVelocity(0)).arg(p.jointVelocity(1)).arg(p.jointVelocity(2)).arg(p.jointVelocity(3))
            .arg(p.jointVelocity(4)).arg(p.jointVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::speedj(Eigen::RowVectorXd jointVelocity, double a, double t)
{
    return QString("speedj([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(jointVelocity(0)).arg(jointVelocity(1)).arg(jointVelocity(2)).arg(jointVelocity(3))
            .arg(jointVelocity(4)).arg(jointVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::speedj(Ur5MovementInfo minfo)
{
    return QString("speedj([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(minfo.targetJointVelocity(0)).arg(minfo.targetJointVelocity(1)).arg(minfo.targetJointVelocity(2))
            .arg(minfo.targetJointVelocity(3)).arg(minfo.targetJointVelocity(4)).arg(minfo.targetJointVelocity(5))
            .arg(minfo.acceleration/1000).arg(minfo.time);
}

QString Ur5MessageEncoder::speedl(Ur5State p, double a, double t)
{
    return QString("speedl([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(p.operationalVelocity(0)).arg(p.operationalVelocity(1)).arg(p.operationalVelocity(2))
            .arg(p.operationalVelocity(3)).arg(p.operationalVelocity(4)).arg(p.operationalVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::speedl(Eigen::RowVectorXd linearVelocity, double a, double t)
{
    return QString("speedl([%1,%2,%3,%4,%5,%6],a=%7,t_min=%8)")
            .arg(linearVelocity(0)).arg(linearVelocity(1)).arg(linearVelocity(2)).arg(linearVelocity(3))
            .arg(linearVelocity(4)).arg(linearVelocity(5)).arg(a).arg(t);
}

QString Ur5MessageEncoder::set_tcp(Ur5State p)
{
    return QString("set_tcp(p[%1,%2,%3,%4,%5,%6])")
            .arg(p.cartAxis(0)).arg(p.cartAxis(1)).arg(p.cartAxis(2)).arg(p.cartAngles(0))
            .arg(p.cartAngles(1)).arg(p.cartAngles(2));
}

QString Ur5MessageEncoder::set_tcp(Transform3D eMt)
{
    if(similar(eMt,Transform3D::Identity()))
    {
        return QString("set_tcp(p[0,0,0,0,0,0])");
    }
    else
    {
        Eigen::RowVectorXd tcp = Ur5Kinematics::T2OperationalConfiguration(eMt);
        return QString("set_tcp(p[%1,%2,%3,%4,%5,%6])")
                .arg(tcp(0)/1000).arg(tcp(1)/1000).arg(tcp(2)/1000).arg(tcp(3))
                .arg(tcp(4)).arg(tcp(5));
    }
}

QString Ur5MessageEncoder::stopl(double a)
{
    return QString("stopl(%1)").arg(a);
}

QString Ur5MessageEncoder::stopj(double a)
{
    return QString("stopj(%1)").arg(a);
}

QString Ur5MessageEncoder::stopj(Ur5MovementInfo minfo)
{
    return QString("stopj(%1)").arg(minfo.acceleration);
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

} // cx
