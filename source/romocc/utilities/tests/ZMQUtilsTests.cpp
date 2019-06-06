//
// Created by androst on 06.06.19.
//

#include <cstring>
#include "romocc/tests/catch.hpp"
#include "romocc/utilities/ZMQUtils.h"

TEST_CASE("Send message manual approach", "[romocc][ZMQUtils]") {
    ZMQUtils::createContext();
    auto streamer = zmq_socket(ZMQUtils::getContext(), ZMQ_STREAM);

    // Send motion command **
    int rc = zmq_connect(streamer, "tcp://localhost:30003");
    unsigned char buffer[1044];
    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);
    char msg [] = "movel(p[-0.020114,-0.431763,0.288153,-0.001221,3.116276,0.038892], a=0.3, v=0.05, r=0)\n";
    zmq_send(streamer, id, id_size, ZMQ_SNDMORE);
    zmq_send(streamer, msg, strlen(msg), 0);
    // **
}
