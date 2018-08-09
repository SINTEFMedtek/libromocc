//
// Created by androst on 02.07.18.
//

#include <utilities/corahHeaders.h>

#include "robotics/RobotState.h"

#ifndef CORAH_MESSAGEDECODER_H
#define CORAH_MESSAGEDECODER_H

struct JointState
{
    Eigen::RowVectorXd jointConfiguration;
    Eigen::RowVectorXd jointVelocity;
    double timestamp = 0;
};

class MessageDecoder {
public:
    virtual JointState analyzeTCPSegment(QByteArray packet) = 0;

    QByteArray slicePacket(QByteArray data, int pos, int length);
    QByteArray getHeader(QByteArray data,int pos);
    QByteArray removeHeader(QByteArray data);

    double pickDouble(QByteArray data, int pos);
    int pickInteger(QByteArray data, int pos);
    int headerLength(QByteArray data);
    int headerID(QByteArray data);

};


#endif //CORAH_MESSAGEDECODER_H
