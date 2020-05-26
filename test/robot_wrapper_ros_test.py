#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import os
import rospy

from robot_control.robot_wrapper_ros import RobotWrapperRos

rospy.init_node("robot_wrapper_ros_test")
ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "assets", )
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")
robot = RobotWrapperRos()
robot.init_from_urdf(URDF_PATH)
robot.init_joint_structure()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    robot.publish_ros()
    rate.sleep()
