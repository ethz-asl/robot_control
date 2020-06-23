cmake_minimum_required(VERSION 3.10)
project(robot_control_ros)

find_package(Eigen3 3.3 REQUIRED)
find_package(Franka REQUIRED)
find_package(pinocchio REQUIRED)

set(PROJECT_DEPENDENCIES
  robot_control
  robot_control_assets
  controller_interface
  franka_hw
  hardware_interface
  pluginlib
  roscpp
  realtime_tools
  interactive_markers
)

find_package(catkin REQUIRED COMPONENTS ${PROJECT_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rc_ros
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
)

set(DEPENDENCY_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${COMMON_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${FRANKA_LIBRARIES})

include_directories(include SYSTEM ${DEPENDENCY_INCLUDE_DIRS})

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(rc_ros ${SRC_FILES})
target_link_libraries(rc_ros ${COMMON_LIBRARIES})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install library
install(TARGETS rc_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

