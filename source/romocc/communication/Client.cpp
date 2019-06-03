#include <sstream>
#include <thread>
#include <iostream>
#include <cstring>

#include "zmq.h"
#include "Client.h"

namespace romocc
{

Client::Client()
{
    mContext = zmq_ctx_new();
    mStreamer = zmq_socket(mContext, ZMQ_STREAM);
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

    int rc = zmq_connect(mStreamer, "tcp://localhost:30003");

    uint8_t id [256];
    size_t  id_size = 256;

    rc = zmq_getsockopt(mStreamer, ZMQ_IDENTITY, &id, &id_size);
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
    auto context = zmq_ctx_new();
    auto streamer = zmq_socket(context, ZMQ_STREAM);

    int rc = zmq_connect(streamer, "tcp://localhost:30003");
    byte buffer[1044];
    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);
    //char msg [] = "movel(p[-0.020114,-0.431763,0.288153,-0.001221,3.116276,0.038892], a=0.3, v=0.05, r=0)\n";
    zmq_send(streamer, id, id_size, ZMQ_SNDMORE);
    zmq_send(streamer, package.c_str(), strlen(package.c_str()), 0);
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
    void *ctx = zmq_ctx_new();
    void *publisher = zmq_socket(ctx, ZMQ_PUB);
    zmq_bind(publisher, "tcp://*:5556");

    try{
        while(true){
            zmq_recv(mStreamer, buffer, 1044, 0);
            int packetLength = getMessageSize(buffer);

            if(packetLength == 1044)
            {
                zmq_send(publisher, buffer, 1044, 0);
            }
        }
    }
    catch (std::exception &error){}
}

}