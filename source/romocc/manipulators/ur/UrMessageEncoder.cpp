#include "UrMessageEncoder.h"
#include <stdio.h>
#include <iostream>

namespace romocc
{

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc,
                                          double vel, double t, double rad)
{
    if(typeOfMovement==MotionType::movej)
        return movej(targetConfiguration,acc,vel,t,rad);
    else if(typeOfMovement==MotionType::movep){
        acc = acc/1000; // mm -> m
        vel = vel/1000;
        rad = rad/1000;

        targetConfiguration(Eigen::seq(0, 2)) = targetConfiguration(Eigen::seq(0, 2))/1000;
        return movep(targetConfiguration,acc,vel,t,rad);
    }
    else if(typeOfMovement==MotionType::speedl){
        acc = acc/1000; // mm -> m
        targetConfiguration.head(3) /= 1000; // mm -> m
        return speedl(targetConfiguration,acc,t);
    }
    else if(typeOfMovement == MotionType::speedj)
        return speedj(targetConfiguration,acc,t);
    else if(typeOfMovement==MotionType::servoc)
        return servoc(targetConfiguration, acc, vel, rad);
    else if(typeOfMovement==MotionType::servol){
        targetConfiguration.head(3) /= 1000; // mm -> m
        return servol(targetConfiguration, t);
    }
    else if(typeOfMovement == MotionType::stopj)
        return stopj(acc);
    else if(typeOfMovement == MotionType::stopl)
        return stopl(acc);
    else
        return std::string("Invalid motion type");
}

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::RowVectorXd targetConfiguration, double acc,
                                          double vel, double t, double lookahead_time, double gain)
{
    if(typeOfMovement==MotionType::servoj)
        return servoj(targetConfiguration,acc,vel,t,lookahead_time,gain);
    else
        return std::string("Invalid motion type");
}

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, Eigen::Affine3d pose, double acc, double vel, double t, double radius)
{
    Vector6d vector = TransformUtils::Affine::toVector6D(pose);
    return moveCommand(MotionType::movep, vector, acc, vel, t, radius);
}

std::string UrMessageEncoder::moveCommand(MotionType typeOfMovement, double targetConfig[6], double acc, double vel, double t, double rad)
{
    Eigen::RowVectorXd vector(6, 1);
    vector << targetConfig[0], targetConfig[1], targetConfig[2], targetConfig[3], targetConfig[4], targetConfig[5];
    return moveCommand(typeOfMovement, vector, acc, vel, t, rad);
}

std::string UrMessageEncoder::stopCommand(MotionType typeOfStop, double acc)
{
    acc = acc/1000;
    if(typeOfStop==MotionType::stopj)
        return stopj(acc);
    else if(typeOfStop==MotionType::stopl)
        return stopl(acc);
    else
        return std::string("Invalid stop type");
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

std::string UrMessageEncoder::servoj(Eigen::RowVectorXd jp, double a, double v, double t, double lookahead_time, double gain)
{
    if(mNewSWVersion)
        return format("servoj([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=%f,lookahead_time=%f,gain=%f)", jp(0), jp(1), jp(2), jp(3), jp(4), jp(5), a, v, t, lookahead_time, gain);
    return format("servoj([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=%f)", jp(0), jp(1), jp(2), jp(3), jp(4), jp(5), a, v, t);
}

std::string UrMessageEncoder::servol(Eigen::RowVectorXd oc, double t)
{
    return format("servol(p[%f,%f,%f,%f,%f,%f],%f)", oc(0), oc(1), oc(2), oc(3), oc(4), oc(5), t);
}

std::string UrMessageEncoder::servoc(Eigen::RowVectorXd oc, double a, double v, double r)
{
    return format("servoc([%f,%f,%f,%f,%f,%f],a=%f,v=%f,r=%f)", oc(0), oc(1), oc(2), oc(3), oc(4), oc(5), a, v, r);
}

std::string UrMessageEncoder::stopl(double a)
{
    return format("stopl(%f)", a);
}

std::string UrMessageEncoder::stopj(double a)
{
    return format("stopj(%f)", a);
}

std::string UrMessageEncoder::setConfigurableOutput(int pin, bool value)
{
    return format("set_configurable_digital_out(%d,%s)", pin, value ? "True" : "False");
}

std::string UrMessageEncoder::setDigitalOutput(int pin, bool value)
{
    return format("set_digital_out(%d,%s)", pin, value ? "True" : "False");
}

std::string UrMessageEncoder::setToolOutput(int pin, bool value)
{
    return format("set_tool_digital_out(%d,%s)", pin, value ? "True" : "False");
}

std::string UrMessageEncoder::setToolVoltage(int voltage)
{
    return format("set_tool_voltage(%d)", voltage);
}

std::string UrMessageEncoder::setAnalogOutput(int pin, double value)
{
    return format("set_analog_out(%d,%f)", pin, value);
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
        if(major_0 == major_1 & minor_0 > minor_1)
            return true;
    }
    return false;
}

}