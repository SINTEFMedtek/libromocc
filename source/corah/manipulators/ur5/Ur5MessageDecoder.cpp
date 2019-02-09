#include "Ur5MessageDecoder.h"
#include <QDebug>
#include <set>
#include <QDataStream>
#include <iostream>

namespace corah
{

JointState Ur5MessageDecoder::analyzeTCPSegment(QByteArray packet)
{
    JointState state;
    if(headerLength(packet)==812 || headerLength(packet) == 1044)
    {
        state = setRTState(slicePacket(packet,sizeof(int),headerLength(packet)));
    }
    return state;
}

JointState Ur5MessageDecoder::setRTState(QByteArray data)
{
    JointState state;

    state.timestamp = pickDouble(data,0);
    state.jointConfiguration = getJointPositionsRT(data);
    state.jointVelocity = getJointVelocitiesRT(data);

    return state;
}

Eigen::RowVectorXd Ur5MessageDecoder::getJointPositionsRT(QByteArray data)
{
    Eigen::RowVectorXd jp(6);
    for(int i=0;i<6;i++)
    {
        jp(i) = pickDouble(data,(31+i)*sizeof(double));
    }
    return jp;
}

Eigen::RowVectorXd Ur5MessageDecoder::getJointVelocitiesRT(QByteArray data)
{
    Eigen::RowVectorXd jv(6);
    for(int i=0;i<6;i++)
    {
        jv(i) = pickDouble(data,(37+i)*sizeof(double));
    }
    return jv;
}

}