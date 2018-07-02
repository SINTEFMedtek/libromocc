//
// Created by androst on 29.06.18.
//

#ifndef CORAH_ROBOT_H
#define CORAH_ROBOT_H

#include <QObject>

#include <communication/Client.h>


class Robot : public QObject
{
    Q_OBJECT

public:
    Robot();
    ~Robot();

private:
    //CommunicationInterface mCommunicationInterface;
    //
};


#endif //CORAH_ROBOT_H
