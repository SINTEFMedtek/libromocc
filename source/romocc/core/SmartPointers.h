//
// Created by androst on 30.05.19.
//

#ifndef ROMOCC_SMARTPOINTERS_H
#define ROMOCC_SMARTPOINTERS_H

#pragma once
#include <memory>

#define ROMOCC_OBJECT(className)                                  \
    public:                                                     \
        typedef std::shared_ptr<className> pointer;               \
        static std::shared_ptr<className> New() {                       \
            std::shared_ptr<className> smartPtr(new className());   \
            smartPtr->setPtr(smartPtr);                              \
                                                                \
            return smartPtr;                                    \
        }                                                       \
        virtual std::string getNameOfClass() const {            \
            return std::string(#className);                     \
        };                                                      \
        static std::string getStaticNameOfClass() {             \
            return std::string(#className);                     \
        };                                                      \
    private:                                                    \
        void setPtr(className::pointer ptr) {                   \
            mPtr = ptr;                                         \
        }                                                       \


#endif //ROMOCC_SMARTPOINTERS_H
