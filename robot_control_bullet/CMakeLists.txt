cmake_minimum_required(VERSION 3.10)
project(robot_control_bullet)

find_package(catkin REQUIRED COMPONENTS pybind11_catkin)
catkin_package()
catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/simulation/sensors/test_depth_image.test)
endif()

