//
// Created by androst on 05.06.19.
//

#ifndef ROMOCC_ZMQUTILS_H
#define ROMOCC_ZMQUTILS_H

#include <zmq.h>
#include <assert.h>
#include "romocc/core/Object.h"

namespace romocc
{

class ZMQUtils {

    public:
        static void*& getContext()
        {
            assert (zmq_context != (void*) nullptr);
            return zmq_context;
        }

        static bool isInitialized()
        {
            return is_initialized;
        }

        static void createContext()
        {
            zmq_context = zmq_ctx_new ();
            is_initialized = true;
        }
    private:
        static void* zmq_context;
        static bool is_initialized;

};


class ZMQUpdateNotifier
{
    public:
        ZMQUpdateNotifier(std::string address){
            mPublisher = zmq_socket(ZMQUtils::getContext(), ZMQ_PUB);
            zmq_bind(mPublisher, ("inproc://" + address).c_str());
        }

        void broadcastUpdate(std::string message = "state_updated"){
            zmq_msg_t zmqMessage;
            zmq_msg_init_size(&zmqMessage, message.size());
            memcpy(zmq_msg_data(&zmqMessage), message.c_str(), message.size());
            zmq_sendmsg(mPublisher, &zmqMessage, ZMQ_DONTWAIT);
        }

        void close()
        {
            zmq_close(mPublisher);
        }

    private:
        void* mPublisher;
};

class ZMQUpdateSubscriber
{
    public:
        ZMQUpdateSubscriber(std::string address, uint32_t buffersize){
            mSubscriber = zmq_socket(ZMQUtils::getContext(), ZMQ_SUB);
            zmq_connect(mSubscriber, ("inproc://" + address).c_str());
            zmq_setsockopt(mSubscriber, ZMQ_SUBSCRIBE, "", 0);
            mBufferSize = buffersize;
            mBuffer = new unsigned char[buffersize]();
        }

        unsigned char* wait_for_update(std::string message = "state_updated"){
            zmq_recv(mSubscriber, mBuffer, mBufferSize, 0);
        }

    private:
        void* mSubscriber;
        unsigned char* mBuffer;
        uint32_t mBufferSize;
};



}

#endif //ROMOCC_ZMQUTILS_H
