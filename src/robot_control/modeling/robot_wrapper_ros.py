# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch


import rospy
import numpy as np
import pinocchio as pin
from sensor_msgs.msg import JointState

from robot_control.modeling import RobotWrapper


class RobotWrapperRos(RobotWrapper):
    """ Adds ros functionality to RobotWrapper """
    def __init__(self):
        super(RobotWrapperRos, self).__init__()
        self.joint_id_map = []
        self.joint_state = JointState()
        self.joint_state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=100)
        self.initialized = False

    def init_from_param(self, param_name):
        """
        Initialize the model and data structures
        :return:
        """
        if not rospy.has_param(param_name):
            raise NameError("Parameter {} does not exist".format(param_name))
        xml_string = rospy.get_param(param_name)
        self.model = pin.buildModelFromUrdf(xml_string)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.v = np.array([0.0] * self.model.nv)
        self.tau = np.array([0.0] * self.model.nv)
        self.nq = self.model.nq
        self.nv = self.model.nv

    def init_joint_structure(self):
        for joint_name in self.model.names:
            if joint_name != "universe":
                self.joint_id_map.append(self.model.getJointId(joint_name))
                self.joint_state.name.append(joint_name)
                self.joint_state.position.append(0.0)
                self.joint_state.velocity.append(0.0)
                self.joint_state.effort.append(0.0)
        self.initialized = True

    def fill_joint_state(self):
        if not self.initialized:
            self.init_joint_structure()

        for idx, joint_id in enumerate(self.joint_id_map):
            q = self.get_q()[joint_id - 1]
            v = self.get_v()[joint_id - 1]
            self.joint_state.position[idx] = q
            self.joint_state.velocity[idx] = v
        self.joint_state.header.stamp = rospy.get_rostime()

    def publish_ros(self):
        self.fill_joint_state()
        self.joint_state_publisher.publish(self.joint_state)

    # TODO (giuseppe) method to get only the gravity terms
    # TODO (giuseppe) method to get only the coriolis terms
