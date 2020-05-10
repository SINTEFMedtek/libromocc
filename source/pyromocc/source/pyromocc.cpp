//
// Created by androst on 31.03.20.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "romocc/Robot.h"
#include "romocc/utilities/MathUtils.h"

namespace py = pybind11;
using namespace romocc;

PYBIND11_MODULE(pyromocc, m) {
    m.doc() = "Python wrapper for the romocc library";

    pybind11::class_<Robot> robot(m, "RobotBase");
    robot.def(py::init<>())
        .def("configure", &Robot::configure)
        .def("connect", &Robot::connect, "Connects to the robot.")
        .def("get_state", &Robot::getCurrentState)
        .def("stop_move", &Robot::stopMove);

    robot.def("movej", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc, double vel){
            self.move(romocc::MotionType::movej, target, acc, vel);
    });

    robot.def("movep", [](Robot& self, Eigen::Ref<const Eigen::MatrixXd> pose, double acc, double vel){
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            self.move(romocc::MotionType::movep, transform, acc, vel);
    });

    pybind11::class_<RobotState, std::shared_ptr<RobotState>> robotState(m, "RobotState");
    robotState.def("get_joint_config", &RobotState::getJointConfig);

    robotState.def("get_pose", [](RobotState& self){
        return self.get_bMee().matrix();
    });

    py::class_<Manipulator> manipulator(m, "Manipulator");

    manipulator.def(py::init<ManipulatorType, std::string>(),
                    py::arg("manipulator_type"), py::arg("sw_version"))
            .def_readwrite("manipulator", &Manipulator::manipulator)
            .def_readwrite("sw_version", &Manipulator::sw_version);

    py::enum_<ManipulatorType>(m, "ManipulatorType")
        .value("UR5", ManipulatorType::UR5)
        .value("UR10", ManipulatorType::UR10);

    py::enum_<MotionType>(m, "MotionType")
        .value("movej", MotionType::movej)
        .value("movep", MotionType::movep)
        .value("speedj", MotionType::speedj)
        .value("speedl", MotionType::speedl)
        .value("stopj", MotionType::stopj)
        .value("stopl", MotionType::stopl);

    m.def("pose_to_vector", [](Eigen::Ref<const Eigen::MatrixXd> pose){
        Eigen::Affine3d transform;
        transform.matrix() = pose;
        return TransformUtils::Affine::toVector6D(transform);
    });

}