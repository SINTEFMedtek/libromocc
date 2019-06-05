//
// Created by androst on 05.06.19.
//

#ifndef ROMOCC_ZMQUTILS_H
#define ROMOCC_ZMQUTILS_H

#include <zmq.h>
#include <assert.h>

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


#endif //ROMOCC_ZMQUTILS_H
