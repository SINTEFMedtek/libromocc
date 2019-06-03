#include "Ur5MessageEncoder.h"

namespace romocc
{

std::string Ur5MessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc,
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

std::string Ur5MessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::Affine3d pose, double acc, double vel, double t, double radius)
{
    pose.translation() = pose.translation()/1000;
    Vector6d vector = AffineToVector6d(pose);
    return moveCommand(MotionType::movep, vector, acc, vel, t, radius);
}

std::string Ur5MessageEncoder::stopCommand(MotionType typeOfStop, double acc)
{
    acc = acc/1000;
    if(typeOfStop==MotionType::stopj)
        return stopj(acc);
    else if(typeOfStop==MotionType::stopl)
        return stopl(acc);
}

std::string Ur5MessageEncoder::shutdownCommand()
{
    return powerdown();
}

std::string Ur5MessageEncoder::movej(Eigen::RowVectorXd q, double a, double v,double t,double r)
{
    return format("movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=%f,r=%f)", q(0), q(1), q(2), q(3), q(4), q(5), a, v, t, r);
}

std::string Ur5MessageEncoder::movep(Eigen::RowVectorXd op,double a, double v,double t,double r)
{
    return format("movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,r=%f)", op(0), op(1), op(2), op(3), op(4), op(5), a, v, r);
}

std::string Ur5MessageEncoder::speedj(Eigen::RowVectorXd jv, double a, double t)
{
    return format("speedj([%f,%f,%f,%f,%f,%f],a=%f,t_min=%f)", jv(0), jv(1), jv(2), jv(3), jv(4), jv(5), a, t);
}

std::string Ur5MessageEncoder::speedl(Eigen::RowVectorXd ov, double a, double t)
{
    return format("speedl([%f,%f,%f,%f,%f,%f],a=%f,t_min=%f)", ov(0), ov(1), ov(2), ov(3), ov(4), ov(5), a, t);
}

std::string Ur5MessageEncoder::stopl(double a)
{
    return format("stopl(%f", a);
}

std::string Ur5MessageEncoder::stopj(double a)
{
    return format("stopj(%f)", a);
}

std::string Ur5MessageEncoder::powerdown()
{
    return std::string("powerdown()");
}

std::string Ur5MessageEncoder::textmsg(std::string str)
{
    return format("textmsg(%s)", str.c_str());
}

std::string Ur5MessageEncoder::sleep(double t)
{
    return std::string("sleep(%f)", t);
}

}