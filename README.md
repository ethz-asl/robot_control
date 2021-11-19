### robot_control 
[![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=robot_control)](https://jenkins.asl.ethz.ch/job/robot_control/)

Clone this repo with the recursive command: `git clone --recursive git@github.com:ethz-asl/robot_control.git` 

## Dependencies

- Install `orocos-kdl`: `sudo apt-get install ros-melodic-orocos-kdl`
- Install `kdl-parser`: `sudo apt-get install ros-melodic-kdl-parser`
- Install `pinocchio`: follow instructions reported [here](https://stack-of-tasks.github.io/pinocchio/download.html)

### Build and Run

`catkin build robot_control robot_control_assets robot_control_bullet robot_control_ros`

`source <catkin_ws>/devel/setup.bash`

### Examples

#### IK Controller
In this example a Inverse Kinematic (IK) Controller computes the preferred
joint configuration for a given desired end effector pose using nullspace to solver for the arm
redundancy. The joint goal configuration is reached using a trapezoidal velocity profile. 
Check the [controller configuration](robot_control_ros/params/controllers.yaml) for more parameters. 
  
`roslaunch robot_control_ros test_controller.launch simulation:=true robot:=kinova3`


### TODO 

Check the Github project!
