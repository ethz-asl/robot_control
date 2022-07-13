cmake_minimum_required(VERSION 3.10)
project(robot_control)

option(CATKIN_BUILD "Unset if building for python 3 through setup.py." ON)

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

if (CATKIN_BUILD)
  find_package(catkin REQUIRED COMPONENTS pybind11_catkin kdl_parser orocos_kdl)
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS pybind11_catkin
    DEPENDS pinocchio)
endif()

add_subdirectory(submodules/pybind11)

set(COMMON_LIBRARIES ${PYTHON_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${BOOST_LIBRARIES})
set(SYSTEM_INCLUDE_DIRECTORIES ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

if(CATKIN_BUILD)
  set(SYSTEM_INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS} ${SYSTEM_INCLUDE_DIRECTORIES})
  set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})
endif()

include_directories(include SYSTEM ${SYSTEM_INCLUDE_DIRECTORIES})

add_library(${PROJECT_NAME}
  src/math/math.cpp
  src/utils/timer.cc
  src/modeling/robot_wrapper.cc
  src/controllers/end_effector_controllers/task_space_controller.cc
  src/controllers/end_effector_controllers/kdl_cartesian_velocity_controller.cc)
target_link_libraries(${PROJECT_NAME}  ${PINOCCHIO_LIBRARIES} ${catkin_LIBRAIRES})

set(SOURCE_BINDINGS_FILES bindings/robot_control.cc)
if (CATKIN_BUILD)
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  pybind_add_module(rc MODULE ${SOURCE_BINDINGS_FILES})
else()
  pybind11_add_module(rc MODULE ${SOURCE_BIDINGS_FILES})
endif()
target_link_libraries(rc PRIVATE ${PROJECT_NAME} ${COMMON_LIBRARIES})
add_dependencies(rc ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

### Testing
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_math unittest/math/math.cpp)
  target_link_libraries(test_math ${PROJECT_NAME})

  catkin_add_gtest(test_cartesian unittest/controllers/end_effector_controllers/test_kdl_cartesian_velocity_controller.cc)
  target_link_libraries(test_cartesian ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(TARGET test_math)
  target_link_libraries(test_math ${catkin_LIBRARIES})
endif()

### Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
