#! /usr/bin/env python

import time
import argparse

import rospy
from std_srvs.srv import Empty

parser = argparse.ArgumentParser()
parser.add_argument('--delay', type=float, default=2.0,
                    help='delay to wait to unpause gazebo physics, default to 2.0s')
args, unknown = parser.parse_known_args()

rospy.init_node("unpause_gazebo_ros")
gazebo_unpause_service = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

rospy.loginfo("Waiting {}s before unpausing gazebo physics".format(args.delay))
time.sleep(args.delay)
try:
    gazebo_unpause_service.call()
except rospy.ROSException as exc:
    rospy.logerr("Unable to unpause gazebo physics, service not available.")
    rospy.logerr(exc)

