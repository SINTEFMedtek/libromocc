//
// Created by androst on 05.06.19.
//

#include "ZMQUtils.h"

namespace romocc
{
    bool ZMQUtils::is_initialized = false;
    void* ZMQUtils::zmq_context = nullptr;
}
