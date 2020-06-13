###! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import rospy
from geometry_msgs.msg import WrenchStamped
import pinocchio as pin
import time

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
import rc
from scipy.spatial.transform import Rotation as Rotation

from robot_control.modeling import RobotWrapperRos
from robot_control.simulation import PybulletRobotWrapper, JointCommandType, PyBulletSimulationBase
from robot_control.simulation import DebugPose
# from robot_control.controllers.implementation import TaskSpaceController
from robot_control.controllers.utilities import TrajectoryGenerator
from rc.controllers import TaskSpaceController

from geometry_msgs.msg import PoseStamped

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets")
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")
MAX_JOINT_FORCE = 50.0

class TestOpSpaceController(PyBulletSimulationBase):
    def __init__(self, time_step):
        super(TestOpSpaceController, self).__init__(time_step=time_step)

        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.1], fixed_base=True)
        self.robot_sim.enable_ft_sensor("end_effector_link")
        self.robot_sim.disable_motors()

        # Manipulator for modeling
        self.wrapper = rc.RobotWrapper(URDF_PATH)
        self.controller = TaskSpaceController(self.wrapper, "end_effector_link")
        self.controller.set_kp([10.0] * 6)
        self.controller.set_kd([10.0] * 6)

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

        # init simulation state
        self.initialize()

    def initialize(self):
        # Initialize structures with current state
        self.robot_sim.set_joint_state(self.initial_q)
        q, v = self.robot_sim.update_state()
        self.controller.advance(np.array(q), np.array(v))

        # Initial position and orientation are used later to compute the target
        R, t, v, w = self.robot_sim.get_link_pose_vel("end_effector_link")
        self.initial_position = t
        self.initial_rotation = R

    def run(self):
        # add debug parameters for ee pose and wrench
        debug_pose = DebugPose()

        user_input = True

        # set the target either from user input or using a circular trajectory
        while True:
            d_pos, d_rot = debug_pose.get_debug_pose()
            target_position = self.initial_position + np.array(d_pos)
            target_rotation = d_rot.dot(self.initial_rotation)
            task_target = rc.SE3(target_rotation, target_position)
            self.controller.set_task_target(task_target)

            # compute and apply control action
            q, v = self.robot_sim.update_state()
            tau_vec = self.controller.advance(q, v)
            for idx, tau in enumerate(tau_vec):
                self.robot_sim.set_command("joint_%d" % (idx+1), JointCommandType.TORQUE, np.clip(tau, -MAX_JOINT_FORCE, MAX_JOINT_FORCE))

            self.robot_sim.send_commands()
            self.step_simulation()


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = TestOpSpaceController(time_step=0.005)
    try:
        test.run()
    except rospy.ROSInterruptException:
        pass
