#!/bin/bash -e
echo "Running the prepare script for robot_control.";
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17

sudo apt-get install -y ros-melodic-libfranka ros-melodic-franka-ros


echo "Installing pinocchio"
sudo apt install robotpkg-py27-pinocchio
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH

#cd robot_control/submodules/pinocchio
#git checkout v2.4.6
#mkdir build && cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
#make -j4
#make install 

