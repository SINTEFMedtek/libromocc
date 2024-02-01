//
// Created by androst on 05.06.19.
//

#ifndef ROMOCC_ZMQUTILS_H
#define ROMOCC_ZMQUTILS_H

#include <zmq.h>
#include <assert.h>

#include "romocc/core/Object.h"
#include "romocc/core/PortableEndian.h"

namespace romocc
{

class ROMOCC_EXPORT ZMQUtils {

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


template<typename T>
static T toLittleEndian(T *big_endian_number)
{
    T little_endian_number;
    char *big_endian_data    = reinterpret_cast<char*>(big_endian_number);
    char *little_endian_data = reinterpret_cast<char*>(&little_endian_number);

    size_t length = sizeof(T);
    for (size_t i = 0; i < length; ++i)
    {
        little_endian_data[i] = big_endian_data[length - i - 1];
    }
    return little_endian_number;
}

static int packageSize(unsigned char* buffer)
{
    std::stringstream ss;
    unsigned int x;
    for(int i=0; i<sizeof(int); i++)
        ss << std::hex << (int)buffer[i];
    ss >> x;

    return static_cast<int>(x);
}

static double* arrayToLittleEndian(double* array, unsigned int N = 6)
{
    for(unsigned int i = 0; i < N; i++)
        array[i] = toLittleEndian(&array[i]);
    return array;
}

static double ntohd(uint64_t nf) {
    double x;
    nf = be64toh(nf);
    memcpy(&x, &nf, sizeof(x));
    return x;
}


} // namespace romocc

#endif //ROMOCC_ZMQUTILS_H
