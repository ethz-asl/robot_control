cmake_minimum_required(VERSION 3.10)
project(robot_control)

option(CATKIN_BUILD "Unset if building for python 3 through setup.py." TRUE)

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
find_package(PythonInterp REQUIRED)

if (CATKIN_BUILD)
  message(STATUS "Building with catkin.")
  find_package(catkin REQUIRED COMPONENTS pybind11_catkin)
  catkin_package(LIBRARIES
    CATKIN_DEPENDS pybind11_catkin)
endif()

add_subdirectory(submodules/pybind11)

set(COMMON_LIBRARIES ${PYTHON_LIBRARIES} ${PINOCCHIO_LIBRARIES})
set(SYSTEM_INCLUDE_DIRECTORIES ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

if(CATKIN_BUILD)
  set(SYSTEM_INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS} ${SYSTEM_INCLUDE_DIRECTORIES})
  set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES})
endif()

include_directories(include SYSTEM ${SYSTEM_INCLUDE_DIRECTORIES})

set(SOURCE_FILES src/robot_control.cc src/robot_wrapper.cc)
if (CATKIN_BUILD)
  install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  pybind_add_module(rc MODULE ${SOURCE_FILES})
else()
  pybind11_add_module(rc MODULE ${SOURCE_FILES})
endif()
target_link_libraries(rc PRIVATE ${COMMON_LIBRARIES})

### Testing
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_math unittest/math.cpp)
endif()

if(TARGET test_math)
  target_link_libraries(
          test_math
          ${catkin_LIBRARIES}
  )
endif()
