#include <iostream>
#include <string>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "pinocchio/spatial/se3.hpp"
#include "robot_control/modeling/robot_wrapper.h"
#include "robot_control/controllers/end_effector_controllers/task_space_controller.h"

namespace pin = pinocchio;
namespace py = pybind11;
using namespace rc;

using Matrix3 = Eigen::Matrix<double, 3, 3>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using SE3 = pin::SE3Tpl<double>;
using Motion = pin::MotionTpl<double>;

PYBIND11_MODULE(rc, m) {
  py::class_<SE3>(m, "SE3")
    .def(py::init<Matrix3, Vector3>())
    .def_property_readonly("rotation", [](const SE3 &self) {
        return self.rotation();
    })
    .def_property_readonly("translation", [](const SE3 &self) {
        return self.translation();
    })
    .def("actInv", [](const SE3 &self, const SE3 &pose_dest) {
        return self.actInv(pose_dest);
    });

  py::class_<Motion>(m, "Motion")
    .def_property_readonly("linear", [](const Motion &self) {
        return self.linear();
    })
    .def_property_readonly("angular", [](const Motion &self) {
        return self.angular();
    });

  py::class_<RobotWrapper>(m, "RobotWrapper")
    .def(py::init<std::string&>())
    .def(py::init<>())
    .def("init_from_urdf", &RobotWrapper::initFromUrdf)
    .def("init_from_xml", &RobotWrapper::initFromXml)
    .def("get_dof", &RobotWrapper::getDof)
    .def("get_neutral_configuration", &RobotWrapper::getNeutralConfiguration)
    .def("get_random_configuration", &RobotWrapper::getRandomConfiguration)
    .def("get_joint_names", &RobotWrapper::getJointNames)
    .def("get_joint_id", &RobotWrapper::getJointId)
    .def("get_q", &RobotWrapper::getQ)
    .def("get_v", &RobotWrapper::getV)
    .def("get_inertia", &RobotWrapper::getInertia)
    .def("update_state", &RobotWrapper::updateState, py::arg("new_q"), py::arg("new_v"),
        py::arg("update_kinematics") = true)
    .def("get_joint_jacobian", &RobotWrapper::getJointJacobian)
    .def("get_frame_jacobian", &RobotWrapper::getFrameJacobian)
    .def("get_all_frame_jacobians", &RobotWrapper::getAllFrameJacobians)
    .def("get_frame_placement", &RobotWrapper::getFramePlacement)
    .def("get_frame_velocity", &RobotWrapper::getFrameVelocity)
    .def("forward_kinematics", &RobotWrapper::forwardKinematics);

  auto controllers = m.def_submodule("controllers");
  py::class_<TaskSpaceController>(controllers, "TaskSpaceController")
    .def(py::init<std::shared_ptr<RobotWrapper>, std::string&>())
    .def("set_task_target", &TaskSpaceController::setTaskTarget)
    .def("compute_command", &TaskSpaceController::computeCommand)
    .def("advance", &TaskSpaceController::advance)
    .def("set_kp", &TaskSpaceController::setKp)
    .def("set_kd", &TaskSpaceController::setKd);

}

