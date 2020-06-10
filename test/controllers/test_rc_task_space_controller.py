###! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python
import unittest
import pinocchio as pin
import time

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
import rc
from scipy.spatial.transform import Rotation as Rotation

from robot_control.simulation import (PybulletRobotWrapper, JointCommandType,
        PyBulletSimulationBase, PyBulletSimulationMode)
from rc.controllers import TaskSpaceController

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets")
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")
MAX_JOINT_FORCE = 50.0

class TestOpSpaceController(unittest.TestCase, PyBulletSimulationBase):
    def __init__(self, *args, **kwargs):
        PyBulletSimulationBase.__init__(self, time_step=0.005,
                mode=PyBulletSimulationMode.HEADLESS)
        unittest.TestCase.__init__(self, *args, **kwargs)

    @classmethod
    def setUpClass(self):
        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.1], fixed_base=True)
        self.robot_sim.enable_ft_sensor("end_effector_link")
        self.robot_sim.disable_motors()

        # Manipulator for modeling
        self.wrapper = rc.RobotWrapper(URDF_PATH)
        self.controller = TaskSpaceController(self.wrapper, "end_effector_link")
        self.controller.set_kp([10.0] * 6)
        self.controller.set_kd([5.0] * 6)

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
        self._initialize(self)
        self.target = rc.SE3(self.initial_rotation, self.initial_position)

    def _initialize(self):
        # Initialize structures with current state
        self.robot_sim.set_joint_state(self.initial_q)
        q, v = self.robot_sim.update_state()
        self.controller.advance(np.array(q), np.array(v))

        # Initial position and orientation are used later to compute the target
        R, t, v, w = self.robot_sim.get_link_pose_vel("end_effector_link")
        self.initial_position = t
        self.initial_rotation = R

    def test_stays_put(self):
        self.controller.set_task_target(self.target)
        for _ in range(20):
            self._step()
        _, t, _, _ = self.robot_sim.get_link_pose_vel("end_effector_link")
        self.assertLess(np.linalg.norm(self.target.translation - t), 1e-1)

        new_translation = self.initial_position + np.array([0.0, 0.1, -0.3])
        self.target = rc.SE3(self.initial_rotation, new_translation)
        self.controller.set_task_target(self.target)
        _, t, _, _ = self.robot_sim.get_link_pose_vel("end_effector_link")
        for _ in range(200):
            self._step()
        _, t, _, _ = self.robot_sim.get_link_pose_vel("end_effector_link")

        self.assertLess(np.linalg.norm(t - new_translation), 0.15)

    def _step(self):
        q, v = self.robot_sim.update_state()
        tau_vec = self.controller.advance(q, v)
        for idx, tau in enumerate(tau_vec):
            self.robot_sim.set_command("joint_%d" % (idx+1), JointCommandType.TORQUE, np.clip(tau, -MAX_JOINT_FORCE, MAX_JOINT_FORCE))
        self.robot_sim.send_commands()
        self.step_simulation()


if __name__ == "__main__":
    unittest.main()

