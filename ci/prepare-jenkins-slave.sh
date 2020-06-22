#!/bin/bash -e
echo "Running the prepare script for robot_control.";
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17

sudo apt install ros-melodic-libfranka ros-melodic-franka-ros


echo "Installing pinocchio"
cd robot_control/submodules/pinocchio
git checkout v2.4.6
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j4
make install 

export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH =/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH =/usr/local/lib/python2.7/site-packages:$PYTHONPATH

