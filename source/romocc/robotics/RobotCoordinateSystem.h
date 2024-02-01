//
// Created by androst on 06.06.19.
//

#ifndef ROMOCC_ROBOTCOORDINATESYSTEM_H
#define ROMOCC_ROBOTCOORDINATESYSTEM_H

#include "romocc/core/Object.h"
#include "Eigen/Dense"

namespace romocc
{

class ROMOCC_EXPORT RobotCoordinateSystem : public Object
{
    ROMOCC_OBJECT(RobotCoordinateSystem)

    public:
        RobotCoordinateSystem() {
            eeMt = Eigen::Affine3d::Identity();
            rMb = Eigen::Affine3d::Identity();
        }

        ~RobotCoordinateSystem(){};

        Transform3d get_rMb() const;
        Transform3d get_eeMt() const;

        void set_eeMt(Eigen::Affine3d eeMt);
        void set_rMb(Eigen::Affine3d rMb);

    private:
        Transform3d eeMt, rMb;
};

}

#endif //ROMOCC_ROBOTCOORDINATESYSTEM_H
