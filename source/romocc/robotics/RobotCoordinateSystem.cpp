//
// Created by androst on 06.06.19.
//

#include "RobotCoordinateSystem.h"

namespace romocc
{

void RobotCoordinateSystem::set_eeMt(Eigen::Affine3d eeMt)
{
    this->eeMt = eeMt;
}

void RobotCoordinateSystem::set_rMb(Eigen::Affine3d rMb)
{
    this->rMb = rMb;
}

Transform3d RobotCoordinateSystem::get_rMb() const
{
    return rMb;
}

Transform3d RobotCoordinateSystem::get_eeMt() const
{
    return eeMt;
}

} // namespace romocc