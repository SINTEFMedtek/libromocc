//
// Created by androst on 31.03.20.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "romocc/Robot.h"


namespace py = pybind11;
using namespace romocc;

PYBIND11_MODULE(pyromocc, m) {
    m.doc() = "Python wrapper for the romocc library";

    pybind11::class_<Robot> robot(m, "RobotBase");
    robot.def(py::init<>())
        .def("configure", &Robot::configure)
        .def("connect", &Robot::connect)
        .def("get_state", &Robot::getCurrentState)
        .def("stop_move", &Robot::stopMove);

    robot.def("movej", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target,double acc, double vel){
            py::print(target);
            self.move(romocc::MotionType::movej, target, acc, vel);
    });

    pybind11::class_<RobotState, std::shared_ptr<RobotState>>(m, "RobotState")
        .def("get_joint_config", &RobotState::getJointConfig)
        .def("get_pose", &RobotState::get_bMee);

    py::class_<Manipulator> manipulator(m, "Manipulator");

    manipulator.def(py::init<Manipulator::ManipulatorType, std::string>())
            .def_readwrite("manipulator", &Manipulator::manipulator)
            .def_readwrite("sw_version", &Manipulator::sw_version);

    py::enum_<Manipulator::ManipulatorType>(manipulator, "ManipulatorType")
        .value("UR5", Manipulator::ManipulatorType::UR5)
        .value("UR10", Manipulator::ManipulatorType::UR10);

    py::enum_<MotionType>(m, "MotionType")
        .value("movej", MotionType::movej)
        .value("movep", MotionType::movep)
        .value("speedj", MotionType::speedj)
        .value("speedl", MotionType::speedl)
        .value("stopj", MotionType::stopj)
        .value("stopl", MotionType::stopl);

}