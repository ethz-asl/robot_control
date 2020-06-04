cmake_minimum_required(VERSION 3.10)
project(robot_control)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
catkin_package(LIBRARIES)

add_subdirectory(submodules/pybind11)

set(PYTHONLIBS_FOUND TRUE)
set(PYBIND11_PYTHON_VERSION "${PYTHON_VERSION}")
set(PYTHON_PREFIX "${PYTHON_ENV}")
set(PYTHON_INTERPRETER "${PYTHON_ENV}/bin/python")
set(PYTHON_LIBRARIES "${PYTHON_ENV}/lib")
set(PYTHON_INCLUDE_DIRS "${PYTHON_ENV}/include")
set(PYTHON_SITE_PACKAGES "${PYTHON_ENV}/lib/python3.7/site-packages")
set(PYTHON_MODULE_EXTENSION ".so")

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} "${PYTHON_INCLUDE_DIRS}/python3.7m" ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

pybind11_add_module(rc MODULE src/robot_control.cc src/robot_wrapper.cc)
target_link_libraries(rc PRIVATE ${PINOCCHIO_LIBRARIES} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# Massive hack to also add the library to the anaconda environment.
add_custom_command(TARGET rc POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rc> "${PYTHON_ENV}/lib/python3.7/site-packages/rc.so"
)

