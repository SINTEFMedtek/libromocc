#ifndef ROMOCC_UR5KDLDEFINITION_H
#define ROMOCC_UR5KDLDEFINITION_H

#include <math.h>
#include <kdl/chain.hpp>

namespace romocc
{

constexpr double dh_d[6] = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
constexpr double dh_a[6] = {0,-0.42500,-0.39225,0,0,0};
constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

KDL::Chain Ur5Chain(){
    KDL::Chain Ur5;
    for (unsigned int i = 0; i < 6; i++) {
        Ur5.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
    }
    return Ur5;
}

}

#endif //ROMOCC_UR5KDLDEFINITION_H
