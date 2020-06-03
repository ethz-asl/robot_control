#include <iostream>
#include <string>
#include <pybind11/pybind11.h>
#include "robot_wrapper.h"

namespace py = pybind11;

PYBIND11_MODULE(rc, m) {
  py::class_<rc::RobotWrapper>(m, "RobotWrapper")
    .def(py::init<>());
}

