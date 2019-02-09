#include "MessageDecoder.h"
#include <QDataStream>

namespace corah
{

double MessageDecoder::pickDouble(QByteArray data, int index)
{
    double a;
    QDataStream stream(data.mid(index,sizeof(double)));
    stream >> a;
    return a;
}

QByteArray MessageDecoder::getHeader(QByteArray data,int pos)
{
    return data.mid(pos,sizeof(int)+sizeof(char));
}

QByteArray MessageDecoder::slicePacket(QByteArray data,int pos, int length)
{
    return data.mid(pos,length);
}

int MessageDecoder::headerLength(QByteArray data)
{
    return pickInteger(data,0);
}

int MessageDecoder::pickInteger(QByteArray data,int pos)
{
    bool ok;
    return data.mid(pos,sizeof(int)).toHex().toInt(&ok,16);
}

int MessageDecoder::headerID(QByteArray data)
{
    bool ok;
    return data.mid((sizeof(int)),sizeof(char)).toHex().toInt(&ok,16);
}

QByteArray MessageDecoder::removeHeader(QByteArray data)
{
    return data.mid(sizeof(int)+sizeof(char),data.size()-sizeof(int)-sizeof(char));
}

}
