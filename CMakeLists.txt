cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS pybind11_catkin)
catkin_package(LIBRARIES CATKIN_DEPENDS pybind11_catkin)

include_directories(${catkin_INCLUDE_DIRS} include)

pybind_add_module(rc MODULE src/robot_control.cpp)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
