#ifndef CORAH_CLIENT_H
#define CORAH_CLIENT_H

#include <string>

#include "zmq.h"
#include "corahExport.hpp"

namespace corah
{

class CORAH_EXPORT Client
{
    public:
        typedef unsigned char byte;

        explicit Client();

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
        void packageReceived(void* buffer);

        ConnectionInfo mConnectionInfo;
        int mCurrentTimestamp;

        void* mContext;
        void* mStreamer;
        void readPackage();

        void start();
        void bufferReady(){};
};

}

#endif //CORAH_CLIENT_H
