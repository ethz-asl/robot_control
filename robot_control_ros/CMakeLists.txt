cmake_minimum_required(VERSION 3.10)
project(robot_control_ros)

find_package(Eigen3 3.3 REQUIRED)
find_package(Franka REQUIRED)
find_package(pinocchio REQUIRED)

message(STATUS "Building with catkin.")
find_package(catkin REQUIRED COMPONENTS
  robot_control
  controller_interface
  franka_hw
  hardware_interface
  pluginlib
  roscpp)

catkin_package(LIBRARIES)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES} ${PINOCCHIO_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(rc_ros ${SRC_FILES})
target_link_libraries(rc_ros ${COMMON_LIBRARIES})

set(SOURCE_FILES src/robot_control.cc src/robot_wrapper.cc)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

