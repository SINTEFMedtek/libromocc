#include "Client.h"

#include <sstream>
#include <iostream>
#include <cstring>
#include <assert.h>

#include "romocc/utilities/ZMQUtils.h"

#include "zmq.h"

namespace romocc
{

Client::Client()
{
    ZMQUtils::createContext();
}

Client::~Client()
{
    if(!mStopThread){
        mStopThread = true;
        mThread->join();
    }
}

bool Client::isConnected()
{
    return mConnected;
}

bool Client::requestConnect(std::string host, int port)
{
    mConnectionInfo.host = host;
    mConnectionInfo.port = port;

    mStreamer = zmq_socket(ZMQUtils::getContext(), ZMQ_STREAM);
    zmq_connect(mStreamer, ("tcp://" + mConnectionInfo.host + ":" + std::to_string(mConnectionInfo.port)).c_str());

    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(mStreamer, ZMQ_IDENTITY, &id, &id_size);

    mStopThread = false;
    mThread = std::make_unique<std::thread>(std::bind(&Client::start, this));
    mConnected = true;

    return isConnected();
}

bool Client::requestDisconnect()
{
    zmq_disconnect(mStreamer, ("tcp://" + mConnectionInfo.host + ":" + std::to_string(mConnectionInfo.port)).c_str());
    mStopThread = true;
    mThread->join();
    mConnected = false;
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
    auto streamer = zmq_socket(ZMQUtils::getContext(), ZMQ_STREAM);
    auto rc = zmq_connect(streamer, ("tcp://" + mConnectionInfo.host + ":" + std::to_string(mConnectionInfo.port)).c_str());
    assert(rc == 0);

    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);

    uint8_t buffer[2048];
    auto publisher = zmq_socket(ZMQUtils::getContext(), ZMQ_PUB);
    rc = zmq_bind(publisher, "inproc://raw_buffer");
    assert(rc == 0);

    while(!mStopThread){
        zmq_recv(streamer, buffer, 2048, 0);
        auto packetLength = getMessageSize(buffer);
        if(packetLength>0 && packetLength<2048)
            zmq_send(publisher, buffer, packetLength, 0);
    }

    zmq_close(publisher);
    zmq_close(streamer);
}

}