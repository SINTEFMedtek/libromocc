#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

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
        .def("get_state", &Robot::getCurrentState)
        .def("stop_move", &Robot::stopMove)
        .def("_connect", &Robot::connect, "Connects to the robot.");

    robot.def("_movej", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc, double vel,
            double time = 0, double blendRad = 0, bool wait=false){
            self.move(romocc::MotionType::movej, target, acc, vel, time, blendRad, wait);
    });

    robot.def("_movep", [](Robot& self, Eigen::Ref<const Eigen::MatrixXd> pose, double acc, double vel,
            double time = 0, double blendRad = 0, bool wait=false){
        if(pose.rows() == 4 && pose.cols() == 4){
            Eigen::Affine3d transform;
            transform.matrix() = pose;
            self.move(romocc::MotionType::movep, transform, acc, vel, time, blendRad, wait);
        } else{
            self.move(romocc::MotionType::movep, pose.transpose(), acc, vel, time, blendRad, wait);
        }
    });

    robot.def("_speedj", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc=M_PI_4, double time=0.5){
        self.move(romocc::MotionType::speedj, target, acc, 0, time);
    });

    robot.def("_speedl", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc=500, double time=0.5){
        self.move(romocc::MotionType::speedl, target, acc, 0, time);
    });

    robot.def("_servoj", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc=500, double vel=500, double time=1/125., double lookahead_time=0.1, double gain=300){
        self.move(romocc::MotionType::servoj, target, acc, vel, time, lookahead_time, gain);
    });

    robot.def("_servol", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double time=1/125.){
        self.move(romocc::MotionType::servol, target, 0, 0, time);
    });

    robot.def("_servoc", [](Robot& self, Eigen::Ref<const Eigen::RowVectorXd> target, double acc=500, double vel=500, double rad=10){
        self.move(romocc::MotionType::servoc, target, acc, vel, rad);
    });

    robot.def("_stopj", [](Robot& self, double acc=M_PI_2){
        self.stopMove(romocc::MotionType::stopj, acc);
    });

    robot.def("_stopl", [](Robot& self, double acc=500){
        self.stopMove(romocc::MotionType::stopl, acc);
    });

    robot.def("wait", [](Robot& self, const std::chrono::steady_clock::time_point &t_cycle_start, double dt=1/125.){
        self.wait(t_cycle_start, dt);
    });

    robot.def("current_time", &Robot::currentTime);

    robot.def("_send_program", &Robot::sendProgram);

    pybind11::class_<RobotState, std::shared_ptr<RobotState>> robotState(m, "RobotState");
    robotState.def("get_joint_config", &RobotState::getJointConfig);
    robotState.def("get_joint_velocity", &RobotState::getJointVelocity);
    robotState.def("get_operational_config", &RobotState::getOperationalConfig);
    robotState.def("get_operational_velocity", &RobotState::getOperationalVelocity);
    robotState.def("get_operational_force", &RobotState::getOperationalForce);
    robotState.def("get_digital_outputs", &RobotState::getDigitalOutputs);
    robotState.def("get_digital_inputs", &RobotState::getDigitalInputs);
    robotState.def("get_configurable_outputs", &RobotState::getConfigurableOutputs);
    robotState.def("get_configurable_inputs", &RobotState::getConfigurableInputs);
    robotState.def("get_tool_outputs", &RobotState::getToolOutputs);
    robotState.def("get_tool_inputs", &RobotState::getToolInputs);
    robotState.def("get_safety_mode", &RobotState::getSafetyMode);
    robotState.def("get_timestamp", &RobotState::getTimestamp);
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
        return self.getJacobian();
    });
    robotState.def("get_transform_to_joint", [](RobotState& self, int jointNr){
        return self.getTransformToJoint(jointNr).matrix();
    });
    py::class_<Manipulator> manipulator(m, "Manipulator");

    manipulator.def(py::init<ManipulatorType, std::string>(),
                    py::arg("manipulator_type"), py::arg("sw_version"))
            .def_readwrite("manipulator", &Manipulator::manipulator)
            .def_readwrite("sw_version", &Manipulator::sw_version);

    py::enum_<ManipulatorType>(m, "ManipulatorType")
        .value("UR3", ManipulatorType::UR3)
        .value("UR3e", ManipulatorType::UR3e)
        .value("UR5", ManipulatorType::UR5)
        .value("UR5e", ManipulatorType::UR5e)
        .value("UR10", ManipulatorType::UR10)
        .value("UR10e", ManipulatorType::UR10e);

    py::enum_<MotionType>(m, "MotionType")
        .value("movej", MotionType::movej)
        .value("movep", MotionType::movep)
        .value("speedj", MotionType::speedj)
        .value("speedl", MotionType::speedl)
        .value("servoj", MotionType::servoj)
        .value("servol", MotionType::servol)
        .value("servoc", MotionType::servoc)
        .value("stopj", MotionType::stopj)
        .value("stopl", MotionType::stopl);

    m.def("pose_to_vector", [](Eigen::Ref<const Eigen::MatrixXd> pose){
        Eigen::Affine3d transform;
        transform.matrix() = pose;
        return TransformUtils::Affine::toVector6D(transform);
    });

    m.def("vector_to_pose", [](Eigen::Matrix<double, 6, 1> vector){
        return TransformUtils::Affine::toAffine3DFromVector6D(vector).matrix();
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
        return self.X.matrix();
    });
    calibration_matrices.def_property_readonly("pose_y", [](CalibrationMatrices& self){
        return self.Y.matrix();
    });

    py::class_<CalibrationError> calibration_error(m, "CalibrationError");
    calibration_error.def_readonly("translation_error", &CalibrationError::translationError);
    calibration_error.def_readonly("rotation_error", &CalibrationError::rotationError);
    calibration_error.def_readonly("translation_std", &CalibrationError::transStd);
    calibration_error.def_readonly("rotation_std", &CalibrationError::rotStd);

    m.def("load_calibration_file", [](std::string filepath){
        auto cal_affine = romocc::load_calibration_file(filepath);
        return cal_affine.matrix();
    });

    m.def("save_calibration_file", [](std::string filepath, Eigen::Ref<const Eigen::MatrixXd> pose){
        Eigen::Affine3d transform;
        transform.matrix() = pose;
        romocc::save_calibration_file(filepath, transform);
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

    calibration_methods.def("calibration_li", [](std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_a,
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
        return CalibrationMethods::Li(poses_a_affine, poses_b_affine);;
    });


    calibration_methods.def("calibration_park", [](std::vector<Eigen::Ref<const Eigen::MatrixXd>> poses_a,
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
        return CalibrationMethods::Park(poses_a_affine, poses_b_affine);
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