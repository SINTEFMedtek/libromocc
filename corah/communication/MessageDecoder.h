//
// Created by androst on 02.07.18.
//

#include <QObject>

#include "robotics/RobotState.h"

#ifndef CORAH_MESSAGEDECODER_H
#define CORAH_MESSAGEDECODER_H


class MessageDecoder {
public:
    virtual RobotState analyzeRawPacket(QByteArray packet) = 0;
};


#endif //CORAH_MESSAGEDECODER_H
