//
// Created by androst on 29.06.18.
//

#include <algorithm>    // std::copy
#include <iostream>
#include "zmq.h"
#include <vector>
#include <sstream>
#include <cstring>
#include <iterator> // std::ostream_iterator
#include <QByteArray>

typedef unsigned char byte;

uint32_t extract_bits(uint8_t *arr, unsigned int bit_index, unsigned int bit_count)
{
    /* Assert that we are not requested to extract more than 32 bits */
    uint32_t result = 0;
    //assert(bit_count <= sizeof(result)*8 && arr != NULL);

    /* You can additionally check if you are trying to extract bits exceeding the 16 byte range */
    //assert(bit_index + bit_count <= 16 * 8);

    unsigned int arr_id = bit_index / 8;
    unsigned int bit_offset = bit_index % 8;

    if (bit_offset > 0) {
        /* Extract first 'unaligned_bit_count' bits, which happen to be non-byte-aligned.
         * When we do extract those bits, the remaining will be byte-aligned so
         * we will thread them in different manner.
         */
        unsigned int unaligned_bit_count = 8 - bit_offset;

        /* Check if we need less than the remaining unaligned bits */
        if (bit_count < unaligned_bit_count) {
            result = (arr[arr_id] >> bit_offset) & ((1 << bit_count) - 1);
            return result;
        }

        /* We need them all */
        result = arr[arr_id] >> bit_offset;
        bit_count -= unaligned_bit_count;

        /* Move to next byte element */
        arr_id++;
    }

    while (bit_count > 0) {
        /* Try to extract up to 8 bits per iteration */
        int bits_to_extract = bit_count > 8 ? 8 : bit_count;

        if (bits_to_extract < 8) {
            result = (result << bits_to_extract) | (arr[arr_id] & ((1 << bits_to_extract)-1));
        }else {
            result = (result << bits_to_extract) | arr[arr_id];
        }

        bit_count -= bits_to_extract;
        arr_id++;
    }

    return result;
}

double pickDouble(uint8_t *data, int index)
{
    uint32_t sliced_data = extract_bits(data, 0, sizeof(int));

    int a;
    memcpy(&a, &sliced_data, sizeof(int));
    std::cout << a << sliced_data << std::endl;
    return 0;
}

int parts(unsigned char* array, int start, int end, int steps)
{
    int i, j;
    char temp [(end-start)/steps + 1];
    for (i = start , j = 0 ; i <= end ; i += steps , ++j)
        temp[j] = array[i];
    for (i = 0 ; i < (end - start)/steps + 1 ; ++i)
        array[i] = temp[i];
    //for ( ; i <= end ; ++i)
    //      array[i] = ' ';
    return (end - start)/steps + 1;
}

int main(int argc, char *argv[])
{
    auto context = zmq_ctx_new();
    auto streamer = zmq_socket(context, ZMQ_STREAM);
    int rc = zmq_connect(streamer, "tcp://localhost:30003");
    byte buffer[1044];
    uint8_t id [256];
    size_t  id_size = 256;
    zmq_getsockopt(streamer, ZMQ_IDENTITY, &id, &id_size);
    char msg [] = "movel(p[-0.020114,-0.431763,0.288153,-0.001221,3.116276,0.038892], a=0.3, v=0.05, r=0)\n";

    //zmq_msg_t zmsg;
    //zmq_msg_init_size(&zmsg, msg.size());
    //memcpy(zmq_msg_data(&zmsg), msg.data(), msg.size());

    //auto bytemsg = zmq_msg_t(void*)msg.c_str(), msg.size()+1, NULL);
    //std::cout << "Sending " << (const char*)bytemsg.data();
    //std::vector<unsigned char> send_buffer(msg.length());
    //msg.copy(&send_buffer[0], send_buffer.size());
    zmq_send(streamer, id, id_size, ZMQ_SNDMORE);
    zmq_send(streamer, msg, strlen(msg), 0);

    bool ok;
    QByteArray databuf;

    int hLength;
    while(1){
        zmq_recv(streamer, buffer, 1044, 0);

        databuf = QByteArray(reinterpret_cast<char*>(buffer), 1044);
        int headerLength = databuf.mid(0,sizeof(int)).toHex().toInt(&ok,16);

        if(headerLength == 1044)
            std::cout << headerLength << std::endl;


        //memcpy(&hLength, buffer+4, sizeof(int));

        //auto sliced_buffer = parts(buffer,0,3,1);
        //memcpy(&hLength, &sliced_buffer, sizeof(int));
        //hLength = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | (buffer[0]);
        //std::cout << &hLength << std::endl;
        //std::cout << &buffer[0] << &buffer[1] << &buffer[2] << &buffer[3] << " " << hLength << std::endl;
        //int i = static_cast<int>(buffer);

    }
}