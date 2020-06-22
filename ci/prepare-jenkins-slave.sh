#!/bin/bash -e
echo "Running the prepare script for robot_control.";
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17

sudo apt-get install -y ros-melodic-libfranka ros-melodic-franka-ros ros-melodic-pybind11-catkin
sudo apt-get install -y python-pip && pip install --user pybullet numpy scipy

echo "Installing pinocchio"
sudo apt-get install -y robotpkg-py27-pinocchio
printenv PATH

#cd robot_control/submodules/pinocchio
#git checkout v2.4.6
#mkdir build && cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
#make -j4
#make install 

