//
// Created by androst on 29.06.18.
//

#ifndef CORAH_COMMUNICATIONINTERFACE_H
#define CORAH_COMMUNICATIONINTERFACE_H

#include <QObject>

#include "communication/Client.h"

class CommunicationInterface : public QObject
{
    Q_OBJECT

public:
    CommunicationInterface();
    ~CommunicationInterface();

    bool sendMessage(QString message);

    bool connectToRobot(QString host, int port);

private:
    //MessageEncoder mEncoder;
    //MessageDecoder mDecoder;
    Client mClient;
};


#endif //CORAH_COMMUNICATIONINTERFACE_H
