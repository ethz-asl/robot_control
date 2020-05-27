#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import os
import math
import numpy as np
import pinocchio as pin

from robot_control.robot_wrapper import RobotWrapper

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")


# Pinocchio based model and controller
wrapper = RobotWrapper()
wrapper.init_from_urdf(URDF_PATH)

initial_q = np.array([0.0] * 7) # np.array([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])
initial_v = np.array([0] * wrapper.get_dof())

wrapper.update_state(initial_q, initial_v)
wrapper.compute_all_terms()


M = wrapper.get_inertia()
J, dJ = wrapper.get_all_frame_jacobians("bracelet_link", pin.ReferenceFrame.LOCAL)
M_inv = wrapper.get_inertia_inverse()
M_ee = np.linalg.pinv(np.dot(J, np.dot(M_inv, J.T)), rcond=0.001)
np.set_printoptions(precision=2)
print("J: \n" + str(J))
print("M_ee:  \n" + str(M_ee))
print(" ")

M = wrapper.get_inertia()
J, dJ = wrapper.get_all_frame_jacobians("end_effector_link", pin.ReferenceFrame.LOCAL)
M_inv = wrapper.get_inertia_inverse()
M_ee = np.linalg.pinv(np.dot(J, np.dot(M_inv, J.T)), rcond=0.001)
np.set_printoptions(precision=2)
print("J: \n" + str(J))
print("M_ee:  \n" + str(M_ee))


