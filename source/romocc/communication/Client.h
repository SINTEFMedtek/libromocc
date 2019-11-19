#ifndef ROMOCC_CLIENT_H
#define ROMOCC_CLIENT_H

#include <string>
#include <thread>

#include "romocc/core/Object.h"
#include "zmq.h"
#include "romoccExport.hpp"

namespace romocc
{

class ROMOCC_EXPORT Client : public Object
{
    ROMOCC_OBJECT(Client)

    public:
        typedef unsigned char byte;

        explicit Client();
        ~Client();

        bool requestConnect(std::string ip_address, int port);
        bool requestDisconnect();

        bool isConnected();
        bool sendPackage(std::string package);

        struct ConnectionInfo
        {
            std::string host;
            int port;
        };

    private:
        int getMessageSize(unsigned char* buffer);

        ConnectionInfo mConnectionInfo;
        std::unique_ptr<std::thread> mThread;
        bool mStopThread = false;
        bool mConnected = false;

        void* mStreamer;
        void start();
};

}

#endif //ROMOCC_CLIENT_H
