#! /home/giuseppe/Programs/anaconda/envs/pybullet_gym/bin/python

import rospy
from geometry_msgs.msg import WrenchStamped
import pinocchio as pin

import time

import os
import math
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as Rotation

from robot_control.modeling import RobotWrapperRos
from robot_control.controllers.implementation import MotionForceControllerRos
from robot_control.controllers.utilities import TrajectoryGenerator

from geometry_msgs.msg import PoseStamped

p.connect(p.GUI)
ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")


def get_link_name_idx_map(obj_id):
    link_idx_map = {p.getBodyInfo(obj_id)[0].decode('UTF-8'): -1, }
    for idx in range(p.getNumJoints(obj_id)):
        name = p.getJointInfo(obj_id, idx)[12].decode('UTF-8')
        link_idx_map[name] = idx
    return link_idx_map


class ManipulatorTest:
    def __init__(self):

        # Manipulator
        self.robot = p.loadURDF(URDF_PATH,
                                useFixedBase=True,
                                basePosition=[0.0, 0.0, 0.0],
                                flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE)

        self.table = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                                basePosition=[0.5, 0, -0.65],
                                useFixedBase=True)
        self.object = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "random_urdfs/000/000.urdf"),
                                 basePosition=[0.5, 0, 0.1])

        print(get_link_name_idx_map(self.table))

        self.dof = p.getNumJoints(self.robot)
        self.q = np.ndarray((7, ), dtype=float)
        self.v = np.ndarray((7, ), dtype=float)
        self.tau = np.ndarray((7,), dtype=float)
        print("The robot has {} dofs".format(self.dof))

        # Pinocchio based model and controller
        self.wrapper = RobotWrapperRos()
        self.wrapper.init_from_urdf(URDF_PATH)
        self.wrapper.init_joint_structure()
        self.controller = MotionForceControllerRos(self.wrapper, "end_effector_link")

        # Other vars
        self.timestep = 0.01  # 1/240.
        self.slowdown_factor = 1
        self.link_map = get_link_name_idx_map(self.robot)
        self.ee_idx_bullet = self.link_map["end_effector_link"]

        self.initial_q = np.array([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])

        self.initial_rotation = None
        self.initial_position = None
        self.prev_time = time.time()
        print(self.link_map)

        # drawing
        self.prev_pose = None
        self.has_prev_pos = False

        # end effector wrench
        p.enableJointForceTorqueSensor(self.robot, self.ee_idx_bullet-2)
        self.measured_wrench = WrenchStamped()
        self.measured_wrench_publisher = rospy.Publisher("bullet/ee_wrench", WrenchStamped, queue_size=10)

    def init_sim(self):
        self.disable_motors()
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.timestep)
        p.setRealTimeSimulation(False)

    def sim(self):
        # Initialize structures with current state
        for i, q in enumerate(self.initial_q):
            p.resetJointState(self.robot, i+1, q)
        self.update_state()
        self.controller.advance(self.q, self.v)

        # Initial position and orientation are used later to compute the target
        R, t, v, w = self.forward_kinematics_bullet()
        self.initial_position = t
        self.initial_rotation = R

        # add debug parameters for ee pose
        dxc = p.addUserDebugParameter("dx", -1.0, 1.0, 0.0)
        dyc = p.addUserDebugParameter("dy", -1.0, 1.0, 0.0)
        dzc = p.addUserDebugParameter("dz", -1.0, 1.0, 0.0)
        drollc = p.addUserDebugParameter("droll", -180.0, 180.0, 0.0)
        dpitchc = p.addUserDebugParameter("dpitch", -180.0, 180.0, 0.0)
        dyawc = p.addUserDebugParameter("dyaw", -180.0, 180.0, 0.0)

        # add debug parameters for ee force
        Fzc = p.addUserDebugParameter("Fz", -10.0, 10.0, 0.0)

        # do not track z in ee frame but force instead
        Sm = np.eye(6)
        Sm[2][2] = 0
        Sf = np.eye(6) - Sm
        self.controller.set_Sm(Sm)
        self.controller.set_Sf(Sf)

        traj_gen = TrajectoryGenerator()
        traj_gen.reset()
        user_input = True

        # publisher for the current desired pose
        desired_pose_publisher = rospy.Publisher("/desired_ee_pose", PoseStamped, queue_size=10)
        desired_pose = PoseStamped()

        task_target = pin.SE3(self.initial_rotation, self.initial_position)
        while True:
            # set the target either from user input or using a circular trajectory
            if user_input:
                dx = p.readUserDebugParameter(dxc)
                dy = p.readUserDebugParameter(dyc)
                dz = p.readUserDebugParameter(dzc)
                droll = p.readUserDebugParameter(drollc)
                dpitch = p.readUserDebugParameter(dpitchc)
                dyaw = p.readUserDebugParameter(dyawc)
                Fz = p.readUserDebugParameter(Fzc)
                target_wrench = np.zeros(6)
                target_wrench[2] = Fz
                target_position = self.initial_position + np.array([dx, dy, dz])
                # add delta rotation in world frame to ee
                dR = Rotation.from_euler(seq='xyz', angles=[droll, dpitch, dyaw], degrees=True)
                target_rotation = dR.as_matrix().dot(self.initial_rotation)
                task_target = pin.SE3(target_rotation, target_position)
                self.controller.set_task_target(task_target)
                self.controller.set_wrench_target(target_wrench)
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
            tau = self.controller.advance(self.q, self.v)
            self.apply_motor_torques(tau)

            self.advance_simulation()

    def advance_simulation(self):
        p.stepSimulation()
        self.draw_trajectory()
        self.get_contact_wrench()
        self.wrapper.publish_ros()
        self.update_state()
        self.report_time()

    def report_time(self):
        """ Sleep additional time if loop is faster """
        # TODO (giuseppe) do some proper reporting
        t = time.time()
        dt = t - self.prev_time
        if dt < self.timestep:
            time.sleep(dt)
        self.prev_time = t

    def draw_trajectory(self):
        """ Plot the trace of the end effector """
        ls = p.getLinkState(self.robot, self.ee_idx_bullet)  # ls[4] world link frame position

        if self.has_prev_pos:
            p.addUserDebugLine(self.prev_pose, ls[4], [1, 0, 0], 1)
        self.prev_pose = ls[4]
        self.has_prev_pos = True

    def update_state(self):
        """ Read the joints positions and velocities from bullet simulator """
        states = p.getJointStates(self.robot, range(1, 8))
        for idx, state in enumerate(states):
            self.q[idx] = state[0]
            self.v[idx] = state[1]
            self.tau[idx] = state[3]

    def forward_kinematics_bullet(self):
        """ Compute the forward kinematic relying on bullet """
        state = p.getLinkState(self.robot, self.ee_idx_bullet, computeForwardKinematics=True,
                               computeLinkVelocity=True)
        t = np.array(state[4])
        R = Rotation.from_quat(state[5])
        R = R.as_dcm()
        v = np.array(state[6])
        w = np.array(state[7])
        return R, t, v, w

    def disable_motors(self):
        """
        Disable the default active motors in each joint and set to zero the default linear and angular
        damping (both equal to 0.04)
        :return:
        """
        p.setJointMotorControlArray(self.robot, range(1, 8), p.VELOCITY_CONTROL, forces=[0.0]*7)
        for i in range(0, 7):
            p.changeDynamics(self.robot, i, linearDamping=0.1, angularDamping=0.1)

    def apply_motor_torques(self, tau):
        assert len(tau) == 7
        p.setJointMotorControlArray(self.robot, range(1, 8), p.TORQUE_CONTROL, forces=tau)

    def get_contact_wrench(self):
        points = p.getContactPoints(self.robot, self.table, self.ee_idx_bullet-1, -1)

        normal_vector_world = np.zeros(3)
        for point in points:
            normal_force = point[9]
            normal_direction = np.array(point[7])
            normal_vector_world += normal_force * normal_direction

        self.measured_wrench.header.stamp = rospy.get_rostime()
        self.measured_wrench.header.frame_id = "base_link"
        self.measured_wrench.wrench.force.x = normal_vector_world[0]
        self.measured_wrench.wrench.force.y = normal_vector_world[1]
        self.measured_wrench.wrench.force.z = normal_vector_world[2]
        self.measured_wrench_publisher.publish(self.measured_wrench)

    def publish_ft_measurements(self):
        # TODO does not seem to work
        joint_state = p.getJointState(self.robot, self.ee_idx_bullet-2)
        link_state = p.getLinkState(self.robot, self.ee_idx_bullet-2, computeForwardKinematics=True)
        r_ij = p.getMatrixFromQuaternion(link_state[-1])
        # r_ij = p.getMatrixFromQuaternion(link_state[1])
        react = joint_state[2]
        # print("measured: " + str(react))
        # print("applied: " + str(joint_state[3]))

        # Rotation world to link frame
        # R_e_w = np.array([r_ij[:3], r_ij[3:6], r_ij[6:]]).transpose()
        R_e_w = np.eye(3)
        w_F = -np.array([react[0], react[1], react[2]])
        w_M = -np.array([react[3], react[4], react[5]])
        e_F = R_e_w.dot(w_F)
        e_M = R_e_w.dot(w_M)
        self.measured_wrench.header.stamp = rospy.get_rostime()
        self.measured_wrench.header.frame_id = "spherical_wrist_2_link"
        self.measured_wrench.wrench.force.x = e_F[0]
        self.measured_wrench.wrench.force.y = e_F[1]
        self.measured_wrench.wrench.force.z = e_F[2]
        self.measured_wrench.wrench.torque.x = e_M[0]
        self.measured_wrench.wrench.torque.y = e_M[1]
        self.measured_wrench.wrench.torque.z = 0 # e_M[2]
        self.measured_wrench_publisher.publish(self.measured_wrench)


if __name__ == "__main__":
    rospy.init_node("control_test")
    test = ManipulatorTest()
    test.init_sim()
    test.sim()
