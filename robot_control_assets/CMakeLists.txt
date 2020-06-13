cmake_minimum_required(VERSION 3.10)
project(robot_control_assets)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


