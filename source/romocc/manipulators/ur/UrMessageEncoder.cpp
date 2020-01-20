#include "UrMessageEncoder.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

namespace romocc
{

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc,
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

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::Affine3d pose, double acc, double vel, double t, double radius)
{
    pose.translation() = pose.translation()/1000;
    Vector6d vector = TransformUtils::Affine::toVector6D(pose);
    return moveCommand(MotionType::movep, vector, acc, vel, t, radius);
}

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, double targetConfig[6], double acc, double vel, double t, double rad)
{
    acc = acc/1000; // mm -> m
    vel = vel/1000;
    rad = rad/1000;

    targetConfig[0] = targetConfig[0]/1000;
    targetConfig[1] = targetConfig[1]/1000;
    targetConfig[2] = targetConfig[2]/1000;

    return movep(targetConfig,acc,vel,t,rad);
}

std::string UrMessageEncoder::stopCommand(MotionType typeOfStop, double acc)
{
    acc = acc/1000;
    if(typeOfStop==MotionType::stopj)
        return stopj(acc);
    else if(typeOfStop==MotionType::stopl)
        return stopl(acc);
}

std::string UrMessageEncoder::shutdownCommand()
{
    return powerdown();
}

std::string UrMessageEncoder::movej(Eigen::RowVectorXd q, double a, double v,double t,double r)
{
    return format("movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=%f,r=%f)", q(0), q(1), q(2), q(3), q(4), q(5), a, v, t, r);
}

std::string UrMessageEncoder::movep(Eigen::RowVectorXd op,double a, double v,double t,double r)
{
    return format("movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,r=%f)", op(0), op(1), op(2), op(3), op(4), op(5), a, v, r);
}

std::string UrMessageEncoder::movep(double op[6], double a, double v, double t, double r)
{
    return format("movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,r=%f)", op[0], op[1], op[2], op[3], op[4], op[5], a, v, r);
}

std::string UrMessageEncoder::speedj(Eigen::RowVectorXd jv, double a, double t)
{
    if(mNewSWVersion)
        return format("speedj([%f,%f,%f,%f,%f,%f],a=%f,t=%f)", jv(0), jv(1), jv(2), jv(3), jv(4), jv(5), a, t);
    return format("speedj([%f,%f,%f,%f,%f,%f],a=%f,t_min=%f)", jv(0), jv(1), jv(2), jv(3), jv(4), jv(5), a, t);
}

std::string UrMessageEncoder::speedl(Eigen::RowVectorXd ov, double a, double t)
{
    if(mNewSWVersion)
        return format("speedl([%f,%f,%f,%f,%f,%f],a=%f,t=%f)", ov(0), ov(1), ov(2), ov(3), ov(4), ov(5), a, t);
    return format("speedl([%f,%f,%f,%f,%f,%f],a=%f,t_min=%f)", ov(0), ov(1), ov(2), ov(3), ov(4), ov(5), a, t);
}

std::string UrMessageEncoder::stopl(double a)
{
    return format("stopl(%f)", a);
}

std::string UrMessageEncoder::stopj(double a)
{
    return format("stopj(%f)", a);
}

std::string UrMessageEncoder::powerdown()
{
    return std::string("powerdown()");
}

std::string UrMessageEncoder::textmsg(std::string str)
{
    return format("textmsg(%s)", str.c_str());
}

std::string UrMessageEncoder::sleep(double t)
{
    return std::string("sleep(%f)", t);
}

void UrMessageEncoder::setSoftwareVersion(std::string version)
{
    mURSoftwareVersion = version;
    mNewSWVersion = compareVersions(mURSoftwareVersion, "3.0");
}

bool UrMessageEncoder::compareVersions(std::string version_0, std::string version_1){
    int major_0, minor_0, major_1, minor_1;
    sscanf(version_0.c_str(), "%d.%d", &major_0, &minor_0);
    sscanf(version_1.c_str(), "%d.%d", &major_1, &minor_1);
    if(major_0 > major_1)
        return true;
    else{
        if(minor_0 > minor_1)
            return true;
    }
    return false;
}

}