//
// Created by androst on 29.06.18.
//

#include <QApplication>
#include "communication/CommunicationInterface.h"
#include "communication/Client.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    // Robot ur5;
    // ur5.setup_manipulator("ur5");
    // ur5.setup_connection(ip_address, port);
    // ur5.initialize();

    CommunicationInterface comInf;

    Client client;

    QString IPaddress = "localhost";
    int port = 30003;

    comInf.connectToRobot(IPaddress, port);

    comInf.sendMessage(QString("movel(p[-130.1,-440.2,-131.9,0.0012,-3.14,-0.05],a=0.1,v=0.1)"));

    return app.exec();
}