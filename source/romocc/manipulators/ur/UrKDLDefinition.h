#ifndef ROMOCC_URKDLDEFINITION_H
#define ROMOCC_URKDLDEFINITION_H

#include <math.h>
#include <kdl/chain.hpp>

// Reference:
// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

namespace romocc
{

namespace Ur5
{
    constexpr double dh_d[6] = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
    constexpr double dh_a[6] = {0,-0.42500,-0.39225,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur5;
        for (unsigned int i = 0; i < 6; i++) {
            Ur5.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur5;
    }

    Eigen::MatrixXd analytic_jacobian(Eigen::RowVectorXd jointConfiguration)
    {
        double s1 = sin(jointConfiguration(0)), c1 = cos(jointConfiguration(0));
        double s2 = sin(jointConfiguration(1)), c2 = cos(jointConfiguration(1));
        double s3 = sin(jointConfiguration(2)), c3 = cos(jointConfiguration(2));
        double s4 = sin(jointConfiguration(3)), c4 = cos(jointConfiguration(3));
        double s5 = sin(jointConfiguration(4)), c5 = cos(jointConfiguration(4));
        double s6 = sin(jointConfiguration(5)), c6 = cos(jointConfiguration(5));

        double s23 = sin(jointConfiguration(1)+jointConfiguration(2));
        double c23 = cos(jointConfiguration(1)+jointConfiguration(2));
        double s234 = sin(jointConfiguration(1)+jointConfiguration(2)+jointConfiguration(3));
        double c234 = cos(jointConfiguration(1)+jointConfiguration(2)+jointConfiguration(3));
        double s2345 = sin(jointConfiguration(1)+jointConfiguration(2)+jointConfiguration(3)+jointConfiguration(4));
        double s234m5 = sin(jointConfiguration(1)+jointConfiguration(2)+jointConfiguration(3)-jointConfiguration(4));

        Eigen::MatrixXd matrix(6,6);

        matrix << dh_d[5]*(c1*c5 + c234*s1*s5) + dh_d[3]*c1 - dh_a[1]*c2*s1 - dh_d[4]*s234*s1 - dh_a[2]*c2*c3*s1 + dh_a[2]*s1*s2*s3,
                -c1*(dh_d[4]*(s23*s4 - c23*c4) + dh_a[2]*s23 + dh_a[1]*s2 - dh_d[5]*s5*(c23*s4 + s23*c4)),
                c1*(dh_d[4]*c234 - dh_a[2]*s23 + dh_d[5]*s234*s5),
                c1*(dh_d[4]*c234 + dh_d[5]*s234*s5),
                dh_d[5]*c1*c2*c5*s3*s4 - dh_d[5]*s1*s5 + dh_d[5]*c1*c3*c5*s2*s4 + dh_d[5]*c1*c4*c5*s2*s3 - dh_d[5]*c1*c2*c3*c4*c5,
                0,
                dh_d[5]*(c5*s1 - c234*c1*s5) + dh_d[3]*s1 + dh_a[1]*c1*c2 + dh_d[4]*s234*c1 + dh_a[2]*c1*c2*c3 - dh_a[2]*c1*s2*s3,
                -s1*(dh_d[4]*(s23*s4 - c23*c4) + dh_a[2]*s23 + dh_a[1]*s2 - dh_d[5]*s5*(c23*s4 + s23*c4)),
                s1*(dh_d[4]*c234 - dh_a[2]*s23 + dh_d[5]*s234*s5),
                s1*(dh_d[4]*c234 + dh_d[5]*s234*s5),
                dh_d[5]*c1*s5 - dh_d[5]*c2*c3*c4*c5*s1 + dh_d[5]*c2*c5*s1*s3*s4 + dh_d[5]*c3*c5*s1*s2*s4 + dh_d[5]*c4*c5*s1*s2*s3,
                0,
                0,
                dh_a[2]*c23 - (dh_d[5]*s2345)/2 + dh_a[1]*c2 + (dh_d[5]*s234m5)/2 + dh_d[4]*s234,
                dh_a[2]*c23 - (dh_d[5]*s2345)/2 + (dh_d[5]*s234m5)/2 + dh_d[4]*s234,
                (dh_d[5]*s234m5)/2 - (dh_d[5]*s2345)/2 + dh_d[4]*s234,
                -dh_d[5]*(s234m5/2 + s2345/2),
                0,
                0,
                s1,
                s1,
                s1,
                s234*c1,
                c5*s1 - c234*c1*s5,
                0,
                -c1,
                -c1,
                -c1,
                s234*s1,
                - c1*c5 - c234*s1*s5,
                1,
                0,
                0,
                0,
                -c234,
                -s234*s5;


        return matrix;
    }
}

namespace Ur10
{
    constexpr double dh_d[6] = {0.1273, 0, 0, 0.163941, 0.1157, 0.0922};
    constexpr double dh_a[6] = {0,-0.612,-0.5723,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur10;
        for (unsigned int i = 0; i < 6; i++) {
            Ur10.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur10;
    }
}

namespace Ur10e
{
    constexpr double dh_d[6] = {0.1807, 0, 0, 0.17415, 0.11985, 0.11655};
    constexpr double dh_a[6] = {0,-0.6127,-0.57155,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur10e;
        for (unsigned int i = 0; i < 6; i++) {
            Ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur10e;
    }
}

namespace Ur3
{
    constexpr double dh_d[6] = {0.1519, 0, 0, 0.11235, 0.08535, 0.0819};
    constexpr double dh_a[6] = {0,-0.24365,-0.21325,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur3;
        for (unsigned int i = 0; i < 6; i++) {
            Ur3.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur3;
    }
}

namespace Ur3e
{
    constexpr double dh_d[6] = {0.15185, 0, 0, 0.13105, 0.08535, 0.0921};
    constexpr double dh_a[6] = {0,-0.24355,-0.2132,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur3e;
        for (unsigned int i = 0; i < 6; i++) {
            Ur3e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur3e;
    }
}

namespace Ur5e
{
    constexpr double dh_d[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    constexpr double dh_a[6] = {0,-0.42500,-0.3922,0,0,0};
    constexpr double dh_alpha[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    constexpr double dh_home[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    KDL::Chain KDLChain(){
        KDL::Chain Ur5e;
        for (unsigned int i = 0; i < 6; i++) {
            Ur5e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(dh_a[i], dh_alpha[i], dh_d[i], 0.0)));
        }
        return Ur5e;
    }
}

KDL::Chain setupKDLChain(Manipulator manipulator) {
    if (manipulator.manipulator == ManipulatorType::UR3){
        return Ur3::KDLChain();
    } else if (manipulator.manipulator == ManipulatorType::UR3e){
        return Ur3e::KDLChain();
    } else if (manipulator.manipulator == ManipulatorType::UR5){
        return Ur5::KDLChain();
    } else if (manipulator.manipulator == ManipulatorType::UR5e){
        return Ur5e::KDLChain();
    } else if (manipulator.manipulator == ManipulatorType::UR10){
        return Ur10::KDLChain();
    } else if (manipulator.manipulator == ManipulatorType::UR10e){
        return Ur10e::KDLChain();
    } else {
        throw std::runtime_error("Manipulator not supported");
    }
}
}

#endif //ROMOCC_URKDLDEFINITION_H
