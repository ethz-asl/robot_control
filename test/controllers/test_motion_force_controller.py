#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

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
from robot_control.simulation import DebugPose, DebugWrench
from robot_control.controllers.implementation import MotionForceControllerRos
from robot_control.controllers.utilities import TrajectoryGenerator

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")


class TestMotionForceController(PyBulletSimulationBase):
    def __init__(self, time_step):
        super(TestMotionForceController, self).__init__(time_step=time_step)

        # Manipulator simulation object
        self.robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
        self.robot_sim.enable_ft_sensor("end_effector_link")
        self.robot_sim.disable_motors()

        # Manipulator for modeling
        self.wrapper = RobotWrapperRos()
        self.wrapper.init_from_urdf(URDF_PATH)
        self.wrapper.init_joint_structure()
        self.controller = MotionForceControllerRos(self.wrapper, "end_effector_link")

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
        debug_wrench = DebugWrench()

        # do not track z in ee frame but force instead
        Sm = np.eye(6)
        Sm[2][2] = 0
        Sf = np.eye(6) - Sm
        self.controller.set_Sm(Sm)
        self.controller.set_Sf(Sf)

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

                force, moment = debug_wrench.get_debug_wrench()
                self.controller.set_wrench_target(np.hstack([force, moment]))
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
            for idx, tau in enumerate(tau_vec):
                self.robot_sim.set_command("joint_%d" % (idx+1), JointCommandType.TORQUE, tau)
            self.robot_sim.send_commands()

            # publish stuff to ros
            self.wrapper.publish_ros()
            self.publish_contact_wrench()
            self.controller.publish_target_wrench()
            self.controller.publish_target_pose()

            self.step_simulation()

    def publish_contact_wrench(self):
        normal_vector_world = self.robot_sim.get_contact_wrench(self.table, "bracelet_link", -1)
        self.measured_wrench.header.stamp = rospy.get_rostime()
        self.measured_wrench.header.frame_id = "base_link"
        self.measured_wrench.wrench.force.x = normal_vector_world[0]
        self.measured_wrench.wrench.force.y = normal_vector_world[1]
        self.measured_wrench.wrench.force.z = normal_vector_world[2]
        self.measured_wrench_publisher.publish(self.measured_wrench)

    # def publish_ft_measurements(self):
    #     # TODO does not seem to work
    #     joint_state = p.getJointState(self.robot, self.ee_idx_bullet-2)
    #     link_state = p.getLinkState(self.robot, self.ee_idx_bullet-2, computeForwardKinematics=True)
    #     r_ij = p.getMatrixFromQuaternion(link_state[-1])
    #     # r_ij = p.getMatrixFromQuaternion(link_state[1])
    #     react = joint_state[2]
    #     # print("measured: " + str(react))
    #     # print("applied: " + str(joint_state[3]))
    #
    #     # Rotation world to link frame
    #     # R_e_w = np.array([r_ij[:3], r_ij[3:6], r_ij[6:]]).transpose()
    #     R_e_w = np.eye(3)
    #     w_F = -np.array([react[0], react[1], react[2]])
    #     w_M = -np.array([react[3], react[4], react[5]])
    #     e_F = R_e_w.dot(w_F)
    #     e_M = R_e_w.dot(w_M)
    #     self.measured_wrench.header.stamp = rospy.get_rostime()
    #     self.measured_wrench.header.frame_id = "spherical_wrist_2_link"
    #     self.measured_wrench.wrench.force.x = e_F[0]
    #     self.measured_wrench.wrench.force.y = e_F[1]
    #     self.measured_wrench.wrench.force.z = e_F[2]
    #     self.measured_wrench.wrench.torque.x = e_M[0]
    #     self.measured_wrench.wrench.torque.y = e_M[1]
    #     self.measured_wrench.wrench.torque.z = 0 # e_M[2]
    #     self.measured_wrench_publisher.publish(self.measured_wrench)


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = TestMotionForceController(time_step=0.005)
    try:
        test.run()
    except rospy.ROSInterruptException:
        pass
