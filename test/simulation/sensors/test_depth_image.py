#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import matplotlib.pyplot as plt

import os
import sys
import math
import numpy as np
import pybullet as p
import pybullet_data

from robot_control.simulation.sensors import CameraConfig, get_camera_image, depth_buffer_to_depth_image
from robot_control.simulation import PybulletRobotWrapper, PyBulletSimulationBase, PyBulletSimulationMode

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "composite", "kinova3_gripper", "kinova3_gripper.urdf")


class TestDepthImage(PyBulletSimulationBase):
    def __init__(self, time_step=None):
        super(TestDepthImage, self).__init__(time_step=time_step, mode=PyBulletSimulationMode.HEADLESS)

        # Populate the scene
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
        self.plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"),
                                basePosition=[0.5, 0, 0.0],
                                useFixedBase=True)
        self.object = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "random_urdfs/000/000.urdf"),
                                 basePosition=[0.5, 0, 0.1])

        # Other vars
        self.initial_q = np.array([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])
        self.robot_sim.set_joint_state(self.initial_q)
        p.setGravity(0, 0, -9.81)

    def run(self):
        config = CameraConfig()
        config.set_fov(70)
        config.set_far_plane(0.4)
        config.set_near_plane(0.1)
        config.set_width(256)
        config.set_height(256)
        print(config)

        test_case = 0
        while True:
            depth_buffer = None
            if test_case == 0:
                # Camera attached to the end effector
                _, _, _, depth_buffer, _ = get_camera_image(config,
                                                            self.robot_sim.get_bullet_id(),
                                                            self.robot_sim.get_link_idx("end_effector_link"),
                                                            position_offset=[0.0, 0.08, 0.02],
                                                            renderer=p.ER_TINY_RENDERER)
            elif test_case == 1:
                # Camera on top looking down
                config.set_far_plane(1.1)
                _, _, _, depth_buffer, _ = get_camera_image(config,
                                                            position_offset=[0.0, 0.0, 1.0],
                                                            orientation_offset=[1, 0, 0, 0],
                                                            renderer=p.ER_TINY_RENDERER)

            depth_image = depth_buffer_to_depth_image(camera_config=config, depth_buffer=depth_buffer)
            plt.imshow(depth_image, cmap='gray', vmin=0, vmax=1)
            plt.title("Depth image")
            plt.show()
            test_case += 1
            if test_case == 2:
                sys.exit(0)
            self.step_simulation()


if __name__ == "__main__":
    test = TestDepthImage()
    test.run()

