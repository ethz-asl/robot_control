# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped

import pinocchio as pin
from scipy.spatial.transform import Rotation

from robot_control.controllers.implementation import MotionForceController


class MotionForceControllerRos(MotionForceController):
    def __init__(self, robot, controlled_frame):
        MotionForceController.__init__(self, robot, controlled_frame=controlled_frame)
        self.target_wrench_ros = WrenchStamped()
        self.target_wrench_pub = rospy.Publisher("~" + self.get_name() + "/target_wrench",
                                                 WrenchStamped,
                                                 queue_size=10)
        self.target_pose_ros = PoseStamped()
        self.target_pose_pub = rospy.Publisher("~" + self.get_name() + "/target_pose",
                                               PoseStamped,
                                               queue_size=10)

    def compute_command(self):
        tau = MotionForceController.compute_command(self)
        self.set_target_wrench_ros(self.get_target_wrench(), self.controlled_frame)
        self.publish_target_wrench()
        return tau

    def set_target_wrench_ros(self, wrench, frame_id):
        assert len(wrench) == 6
        self.target_wrench_ros.header.frame_id = frame_id
        self.target_wrench_ros.header.stamp = rospy.get_rostime()
        self.target_wrench_ros.wrench.force.x = wrench[0]
        self.target_wrench_ros.wrench.force.y = wrench[1]
        self.target_wrench_ros.wrench.force.z = wrench[2]
        self.target_wrench_ros.wrench.torque.x = wrench[3]
        self.target_wrench_ros.wrench.torque.y = wrench[4]
        self.target_wrench_ros.wrench.torque.y = wrench[5]

    def set_target_pose_ros(self, pose, frame_id):
        # TODO (giuseppe) add other input types
        assert isinstance(pose, pin.SE3)
        self.target_pose_ros.header.frame_id = frame_id
        self.target_pose_ros.header.stamp = rospy.get_rostime()
        self.target_pose_ros.pose.position.x = pose.translation[0]
        self.target_pose_ros.pose.position.y = pose.translation[1]
        self.target_pose_ros.pose.position.z = pose.translation[2]
        q = Rotation.from_matrix(pose.rotation).as_quat()
        self.target_pose_ros.pose.orientation.x = q[0]
        self.target_pose_ros.pose.orientation.y = q[1]
        self.target_pose_ros.pose.orientation.z = q[2]
        self.target_pose_ros.pose.orientation.w = q[3]

    def publish_target_wrench(self):
        self.target_wrench_pub.publish(self.target_wrench_ros)

    def publish_target_pose(self):
        self.target_pose_pub.publish(self.target_pose_ros)
