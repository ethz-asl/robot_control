cmake_minimum_required(VERSION 3.10)
project(robot_control_ros)

find_package(Eigen3 REQUIRED 3.3)

message(STATUS "Building with catkin.")
find_package(catkin REQUIRED COMPONENTS pybind11_catkin)
catkin_package(LIBRARIES)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

set(SOURCE_FILES src/robot_control.cc src/robot_wrapper.cc)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

