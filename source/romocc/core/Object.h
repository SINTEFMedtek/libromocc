//
// Created by androst on 02.06.19.
//

#ifndef ROMOCC_OBJECT_H
#define ROMOCC_OBJECT_H

#include "ForwardDeclarations.h"
#include "SmartPointers.h"

namespace romocc{

class Object {
    public:
        typedef SharedPointer<Object> pointer;
        virtual void update() = 0;

    protected:
        std::weak_ptr<Object> mPtr;

};

} // end namespace romocc

#endif //ROMOCC_OBJECT_H
