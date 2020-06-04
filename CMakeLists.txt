cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS pybind11_catkin)
find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
catkin_package(LIBRARIES CATKIN_DEPENDS pybind11_catkin)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

pybind_add_module(rc MODULE src/robot_control.cc src/robot_wrapper.cc)
target_link_libraries(rc PRIVATE ${PINOCCHIO_LIBRARIES} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

