cmake_minimum_required(VERSION 3.10)
project(control_barrier_functions)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
        #        INCLUDE_DIRS include
        #        LIBRARIES
        #        CATKIN_DEPENDS
)
catkin_python_setup()

catkin_install_python(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
