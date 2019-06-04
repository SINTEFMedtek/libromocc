#include <sstream>
#include <thread>
#include <iostream>
#include <cstring>
#include <assert.h>

#include "zmq.h"
#include "Client.h"

namespace romocc
{

Client::Client()
{
    mContext = zmq_ctx_new();
}

bool Client::isConnected()
{
    // Not implemented
    return false;
}

bool Client::requestConnect(std::string host, int port)
{
    mConnectionInfo.host = host;
    mConnectionInfo.port = port;

    mStreamer = zmq_socket(mContext, ZMQ_STREAM);
    zmq_connect(mStreamer, ("tcp://" + host + ":" + std::to_string(port)).c_str());

    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(mStreamer, ZMQ_IDENTITY, &id, &id_size);

    std::thread thread_(std::bind(&Client::start, this));
    thread_.detach();
    return isConnected();
}

bool Client::requestDisconnect()
{
    // Not implemented
    return true;
}

bool Client::sendPackage(std::string package)
{
    uint8_t id [256];
    size_t id_size = 256;
    zmq_getsockopt(mStreamer, ZMQ_IDENTITY, &id, &id_size);
    zmq_send(mStreamer, id, id_size, ZMQ_SNDMORE);
    zmq_send(mStreamer, package.c_str(), strlen(package.c_str()), 0);
    return true;
}

int Client::getMessageSize(unsigned char* buffer)
{
    std::stringstream ss;
    unsigned int x;
    for(int i=0; i<sizeof(int); i++)
        ss << std::hex << (int)buffer[i];
    ss >> x;

    return static_cast<int>(x);
}

void Client::start()
{
    byte buffer[1044];
    auto publisher = zmq_socket(mContext, ZMQ_PUB);
    auto rc = zmq_bind(publisher, "inproc://raw_buffer"); assert(rc == 0);

    try{
        while(true){
            zmq_recv(mStreamer, buffer, 1044, 0);
            auto packetLength = getMessageSize(buffer);

            if(packetLength == 1044)
            {
                zmq_send(publisher, buffer, 1044, 0);
            }
        }
    }
    catch (std::exception &error){}
}

}