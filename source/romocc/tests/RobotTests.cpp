//
// Created by androst on 05.06.19.
//

#include "catch.hpp"
#include "romocc/Robot.h"

namespace romocc {

TEST_CASE("Initialize robot with wrong address and check if connected", "[romocc][Robot]"){
    Robot::pointer robot = Robot::New();
    robot->configure(UR5, "dummy", 9999);
    robot->start();
    CHECK(robot->isConnected() == false);
}

TEST_CASE("Initialize robot and check reference system", "[romocc][Robot]") {
    Robot::pointer robot = Robot::New();
    CHECK(robot->get_rMb().matrix() == Eigen::Affine3d::Identity().matrix());
}

}
