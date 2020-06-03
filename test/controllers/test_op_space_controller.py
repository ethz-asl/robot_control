#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import rospkg
import rospy
import xacro
from geometry_msgs.msg import WrenchStamped
import pinocchio as pin

import math
import os
import tempfile
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as Rotation

from robot_control.modeling import RobotWrapperRos
from robot_control.simulation import PybulletRobotWrapper, JointCommandType, PyBulletSimulationBase
from robot_control.simulation import DebugPose
from robot_control.controllers.implementation import OperationalSpaceControllerRos
from robot_control.controllers.utilities import TrajectoryGenerator

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets",)
USE_JOINTS = 7
URDF_PATH = os.path.join(ASSETS_PATH, 'arms', 'panda', 'panda.urdf')
EE_LINK = 'panda_hand'

class TestOpSpaceController(PyBulletSimulationBase):
    def __init__(self, time_step):
        super(TestOpSpaceController, self).__init__(time_step=time_step)

        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
        self.robot_sim.enable_ft_sensor(EE_LINK)
        self.robot_sim.disable_motors()

        # Manipulator for modeling
        self.wrapper = RobotWrapperRos()
        self.wrapper.init_from_urdf(URDF_PATH)
        self.wrapper.init_joint_structure()
        self.controller = OperationalSpaceControllerRos(self.wrapper, EE_LINK)

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
        R, t, v, w = self.robot_sim.get_link_pose_vel(EE_LINK)
        self.initial_position = t
        self.initial_rotation = R

    def run(self):
        # add debug parameters for ee pose and wrench
        debug_pose = DebugPose()

        traj_gen = TrajectoryGenerator()
        traj_gen.reset()
        user_input = True

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

            # compute and apply control action
            q, v = self.robot_sim.update_state()
            tau_vec = self.controller.advance(q, v)
            for idx, tau in enumerate(tau_vec[:USE_JOINTS]):
                self.robot_sim.set_command("panda_joint%d" % (idx+1), JointCommandType.TORQUE, tau)

            self.robot_sim.send_commands()
            self.wrapper.publish_ros()
            self.controller.publish_target_pose()
            self.step_simulation()


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = TestOpSpaceController(time_step=0.005)
    try:
        test.run()
    except rospy.ROSInterruptException:
        pass

