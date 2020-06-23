//
// Created by androst on 31.03.20.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "romocc/Robot.h"
#include "romocc/utilities/MathUtils.h"
#include "romocc/robotics/RobotMotion.h"

#include "romocc/calibration/CalibrationMethods.h"
#include "romocc/calibration/CalibrationHelpers.h"

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
        if(pose.rows() == 4 && pose.cols() == 4){
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            self.move(romocc::MotionType::movep, transform, acc, vel);
        } else{
            self.move(romocc::MotionType::movep, pose.transpose(), acc, vel);
        }
    });

    robot.def("speedj", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc, double time=5){
        self.move(romocc::MotionType::speedj, target, acc, 0, time);
    });
    robot.def("speedl", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc, double time=5){
        self.move(romocc::MotionType::speedl, target, acc, 0, time);
    });
    robot.def("stopj", [](Robot& self, double acc){
        self.stopMove(romocc::MotionType::stopj, acc);
    });
    robot.def("stopl", [](Robot& self, double acc){
        self.stopMove(romocc::MotionType::stopl, acc);
    });
    pybind11::class_<RobotState, std::shared_ptr<RobotState>> robotState(m, "RobotState");
    robotState.def("get_joint_config", &RobotState::getJointConfig);
    robotState.def("get_joint_velocity", &RobotState::getJointVelocity);
    robotState.def("joint_to_pose", [](RobotState& self, Eigen::Ref<const Eigen::RowVectorXd> joint_config){
        return self.jointConfigToOperationalConfig(joint_config).matrix();
    });

    robotState.def("pose_to_joint", [](RobotState& self, Eigen::Ref<const Eigen::MatrixXd> pose){
        Eigen::Affine3d transform;
        transform.matrix() = pose;
        return self.operationalConfigToJointConfig(transform);
    });

    robotState.def("get_pose", [](RobotState& self){
        return self.get_bMee().matrix();
    });
    robotState.def("get_inverse_jacobian", [](RobotState& self){
        return self.getJacobian().inverse();
    });
    robotState.def("get_jacobian", [](RobotState& self){
        return self.getJacobian().matrix();
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

    py::class_<RobotMotionQueue> motion_queue(m, "MotionQueue");
    motion_queue.def(py::init<>())
        .def("get_queue", &RobotMotionQueue::getQueue);
    motion_queue.def("add", [](RobotMotionQueue& self, Eigen::Ref<const Eigen::MatrixXd> pose, double acc, double vel,
            double time = 0, double blendRadius = 0){
        if(pose.rows() == 4 && pose.cols() == 4){
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            self.add(transform, MotionType::movep, acc, vel, time, blendRadius) ;
        } else{
            std::cout << "Pose should be 4x4." << std::endl;
        }
    });

    py::class_<RobotMotion> robot_motion(m, "RobotMotion");
    robot_motion.def("target_pose", [](RobotMotion& self){
        return self.targetPose.matrix();
    });

    py::class_<CalibrationMatrices> calibration_matrices(m, "CalibrationMatrices");
    calibration_matrices.def_property_readonly("pose_x", [](CalibrationMatrices& self){
        return self.prMb.matrix();
    });
    calibration_matrices.def_property_readonly("pose_y", [](CalibrationMatrices& self){
        return self.eeMt.matrix();
    });

    py::class_<CalibrationError> calibration_error(m, "CalibrationError");
    calibration_error.def_readonly("translation_error", &CalibrationError::translationError);
    calibration_error.def_readonly("rotation_error", &CalibrationError::rotationError);

    m.def("load_calibration_file", [](std::string filepath){
        auto cal_affine = romocc::load_calibration_file(filepath);
        return cal_affine.matrix();
    });

    m.def("save_calibration_file", [](std::string filepath, Eigen::Ref<const Eigen::MatrixXd> pose){
        Eigen::Affine3d transform;
        transform.matrix() = pose;
        save_calibration_file(filepath, transform);
    });

    py::class_<CalibrationMethods> calibration_methods(m, "CalibrationMethods");
    calibration_methods.def("calibration_shah", [](std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_a,
                                                   std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_b){
        std::vector<Eigen::Affine3d> poses_a_affine;
        std::vector<Eigen::Affine3d> poses_b_affine;

        for(auto const& pose: poses_a) {
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            poses_a_affine.push_back(transform);
        }

        for(auto const& pose: poses_b) {
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            poses_b_affine.push_back(transform);
        }
        return CalibrationMethods::Shah(poses_a_affine, poses_b_affine);;
    });

    calibration_methods.def("estimate_calibration_error", [](Eigen::Ref<const Eigen::MatrixXd> pose_x,
                                                             Eigen::Ref<const Eigen::MatrixXd> pose_y,
                                                             std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_a,
                                                             std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_b){
        std::vector<Eigen::Affine3d> poses_a_affine;
        std::vector<Eigen::Affine3d> poses_b_affine;

        for(auto const& pose: poses_a) {
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            poses_a_affine.push_back(transform);
        }

        for(auto const& pose: poses_b) {
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            poses_b_affine.push_back(transform);
        }

        Eigen::Affine3d pose_x_affine;
        pose_x_affine.matrix() = pose_x;

        Eigen::Affine3d pose_y_affine;
        pose_y_affine.matrix() = pose_y;

        return CalibrationMethods::estimateCalibrationError(pose_x_affine, pose_y_affine, poses_a_affine, poses_b_affine);
    });
}