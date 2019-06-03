#include "MessageDecoder.h"

namespace romocc
{

int MessageDecoder::packageSize(unsigned char* buffer)
{
    std::stringstream ss;
    unsigned int x;
    for(int i=0; i<sizeof(int); i++)
        ss << std::hex << (int)buffer[i];
    ss >> x;

    return static_cast<int>(x);
}

double* MessageDecoder::arrayToLittleEndian(double* array, unsigned int N)
{
    for(unsigned int i = 0; i < N; i++)
        array[i] = toLittleEndian(&array[i]);
    return array;
}
}
