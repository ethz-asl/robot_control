#! /usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import pinocchio as pin

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as Rotation

from robot_control.modeling import RobotWrapperRos
from robot_control.simulation import PybulletRobotWrapper, JointCommandType, PyBulletSimulationBase
from robot_control.simulation import DebugPose
from robot_control.controllers.implementation import TaskSpaceController
from robot_control.controllers.utilities import TrajectoryGenerator

from geometry_msgs.msg import PoseStamped

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")


class TestOpSpaceController(PyBulletSimulationBase):
    def __init__(self, time_step):
        super(TestOpSpaceController, self).__init__(time_step=time_step)

        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
        self.robot_sim.enable_ft_sensor("end_effector_link")
        self.robot_sim.disable_motors()

        # Manipulator for modeling
        self.wrapper = RobotWrapperRos()
        self.wrapper.init_from_urdf(URDF_PATH)
        self.wrapper.init_joint_structure()
        self.controller = TaskSpaceController(self.wrapper, "end_effector_link")

        # Other objects in the scene
        self.table = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                                basePosition=[0.5, 0, -0.65],
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
        q, v = self.robot_sim.update_state()
        self.controller.advance(q, v)

        # Initial position and orientation are used later to compute the target
        R, t, v, w = self.robot_sim.get_link_pose_vel("end_effector_link")
        self.initial_position = t
        self.initial_rotation = R

    def run(self):
        # add debug parameters for ee pose and wrench
        debug_pose = DebugPose()

        traj_gen = TrajectoryGenerator()
        traj_gen.reset()
        user_input = True

        # publisher for the current desired pose
        desired_pose_publisher = rospy.Publisher("/desired_ee_pose", PoseStamped, queue_size=10)
        desired_pose = PoseStamped()

        # set the target either from user input or using a circular trajectory
        while True:
            if user_input:
                d_pos, d_rot = debug_pose.get_debug_pose()
                target_position = self.initial_position + np.array(d_pos)
                target_rotation = d_rot.dot(self.initial_rotation)
                task_target = pin.SE3(target_rotation, target_position)
                self.controller.set_task_target(task_target)
            else:
                pos, orn = traj_gen.advance_circular_trajectory(max_angle=np.pi*2.)
                pos = self.initial_position + pos
                pos[0] -= traj_gen.get_radius()
                orn = Rotation.from_quat(orn).as_matrix()
                task_target = pin.SE3(orn, pos)
                self.controller.set_task_target(task_target)

            # publish target over ros
            desired_pose.header.stamp = rospy.get_rostime()
            desired_pose.header.frame_id = "base_link"
            desired_pose.pose.position.x = task_target.translation[0]
            desired_pose.pose.position.y = task_target.translation[1]
            desired_pose.pose.position.z = task_target.translation[2]
            quat = Rotation.from_matrix(task_target.rotation).as_quat()
            desired_pose.pose.orientation.x = quat[0]
            desired_pose.pose.orientation.y = quat[1]
            desired_pose.pose.orientation.z = quat[2]
            desired_pose.pose.orientation.w = quat[3]
            desired_pose_publisher.publish(desired_pose)

            # compute and apply control action
            q, v = self.robot_sim.update_state()
            tau_vec = self.controller.advance(q, v)
            for idx, tau in enumerate(tau_vec):
                self.robot_sim.set_command("joint_%d" % (idx+1), JointCommandType.TORQUE, tau)

            self.robot_sim.send_commands()
            self.wrapper.publish_ros()
            self.step_simulation()


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = TestOpSpaceController(time_step=0.005)
    try:
        test.run()
    except rospy.ROSInterruptException:
        pass
