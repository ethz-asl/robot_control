#include <iostream>
#include <string>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "robot_wrapper.h"

namespace py = pybind11;
using namespace rc;

PYBIND11_MODULE(rc, m) {
  py::class_<RobotWrapper>(m, "RobotWrapper")
    .def(py::init<std::string&>())
    .def("get_dof", &RobotWrapper::getDof)
    .def("get_neutral_configuration", &RobotWrapper::getNeutralConfiguration)
    .def("get_random_configuration", &RobotWrapper::getRandomConfiguration)
    .def("get_q", &RobotWrapper::getQ)
    .def("get_v", &RobotWrapper::getV)
    .def("update_state", &RobotWrapper::updateState, py::arg("new_q"), py::arg("new_v"),
        py::arg("update_kinematics") = true)
    .def("get_joint_jacobian", &RobotWrapper::getJointJacobian)
    .def("get_frame_jacobian", &RobotWrapper::getFrameJacobian)
    .def("forward_kinematics", &RobotWrapper::forwardKinematics);
}

