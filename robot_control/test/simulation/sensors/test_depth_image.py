#! /usr/bin/env python
import unittest
import os
import math
import numpy as np
import pybullet as p
import pybullet_data

from robot_control.simulation.sensors import CameraConfig, get_camera_image, depth_buffer_to_depth_image
from robot_control.simulation import PybulletRobotWrapper

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "composite", "kinova3_gripper", "kinova3_gripper.urdf")


class TestDepthImage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(True)

        cls.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0.5, 0, 0.0], useFixedBase=True)
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "random_urdfs/000/000.urdf"), [0.5, 0, 0.1])

        # Other vars
        initial_q = np.array([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])
        cls.robot_sim.set_joint_state(initial_q)
        p.setGravity(0, 0, -9.81)

    def test_in_hand_camera(self):
        config = CameraConfig(fov=70, width=256, height=128, far_plane=0.4, near_plane=0.1)
        w, h, rgb, depth, seg = get_camera_image(config,
                                                 self.robot_sim.robot,
                                                 self.robot_sim.name_to_idx_map["end_effector_link"],
                                                 position_offset=[0.0, 0.08, 0.02],
                                                 renderer=p.ER_TINY_RENDERER)
        self.assertEqual(w, 256)
        self.assertEqual(h, 128)
        self.assertEqual(rgb.shape[0], 128)
        self.assertEqual(rgb.shape[1], 256)
        self.assertEqual(depth.shape[0], 128)
        self.assertEqual(depth.shape[1], 256)

    def test_static_camera(self):
        config = CameraConfig(fov=70, width=256, height=128, far_plane=1.1, near_plane=0.1)
        w, h, rgb, depth, seg = get_camera_image(config,
                                                 position_offset=[0.0, 0.0, 1.0],
                                                 orientation_offset=[1, 0, 0, 0],
                                                 renderer=p.ER_TINY_RENDERER)

        self.assertEqual(w, 256)
        self.assertEqual(h, 128)
        self.assertEqual(rgb.shape[0], 128)
        self.assertEqual(rgb.shape[1], 256)
        self.assertEqual(depth.shape[0], 128)
        self.assertEqual(depth.shape[1], 256)

    def test_depth_conversion(self):
        config = CameraConfig(fov=70, width=256, height=256, far_plane=1.1, near_plane=0.1)
        _, _, _, depth_buffer, _ = get_camera_image(config,
                                                    position_offset=[0.0, 0.0, 1.0],
                                                    orientation_offset=[1, 0, 0, 0],
                                                    renderer=p.ER_TINY_RENDERER)

        depth_image = depth_buffer_to_depth_image(camera_config=config, depth_buffer=depth_buffer)
        max_depth = np.max(depth_image)
        min_depth = np.max(depth_image)
        self.assertTrue(max_depth <= 1.1)
        self.assertTrue(min_depth >= 0.1)


if __name__ == "__main__":
    unittest.main()
