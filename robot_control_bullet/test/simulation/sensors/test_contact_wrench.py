#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped

import os
import math
import numpy as np
import pybullet as p
import pybullet_data

from robot_control_assets import RobotControlAssets as rca
from robot_control_bullet.simulation import PybulletRobotWrapper, JointCommandType, PyBulletSimulationBase
from robot_control_bullet.simulation.sensors import get_contact_wrench_b_on_a


class TestContactWrench(PyBulletSimulationBase):
    def __init__(self, time_step):
        super(TestContactWrench, self).__init__(time_step=time_step)

        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(rca.get_kinova_urdf_path(),
                                              basePosition=[0.0, 0.0, 0.0],
                                              fixed_base=True)
        self.robot_sim.enable_ft_sensor("end_effector_link")
        self.robot_sim.disable_motors()

        # Other objects in the scene
        self.table = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                                basePosition=[0.5, 0, -0.75],
                                useFixedBase=True)
        self.object = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "random_urdfs/000/000.urdf"),
                                 basePosition=[0.5, 0, 0.1])

        # Other vars
        self.initial_q = np.array([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])
        self.initial_rotation = None
        self.initial_position = None

        # end effector wrench
        self.measured_wrench = WrenchStamped()
        self.measured_wrench_publisher = rospy.Publisher("bullet/ee_wrench", WrenchStamped, queue_size=10)

        # init simulation state
        self.initialize()

    def initialize(self):
        # Initialize structures with current state
        self.robot_sim.set_joint_state(self.initial_q)

        # Initial position and orientation are used later to compute the target
        R, t, v, w = self.robot_sim.get_link_pose_vel("end_effector_link")
        self.initial_position = t
        self.initial_rotation = R

    def run(self):
        while True:
            self.step_simulation()

    def publish_contact_wrench(self):
        wrench = get_contact_wrench_b_on_a(self.robot_sim.robot, self.table,
                                           self.robot_sim.name_to_idx_map["end_effector_link"])
        self.measured_wrench.header.stamp = rospy.get_rostime()
        self.measured_wrench.header.frame_id = "end_effector_link"
        self.measured_wrench.wrench.force.x = wrench[0]
        self.measured_wrench.wrench.force.y = wrench[1]
        self.measured_wrench.wrench.force.z = wrench[2]
        self.measured_wrench.wrench.torque.x = wrench[3]
        self.measured_wrench.wrench.torque.y = wrench[4]
        self.measured_wrench.wrench.torque.z = wrench[5]
        self.measured_wrench_publisher.publish(self.measured_wrench)


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = TestContactWrench(time_step=0.005)
    try:
        test.run()
    except rospy.ROSInterruptException:
        pass
