from __future__ import print_function
import pinocchio as pin

import time

import os
import math
import numpy as np
import pybullet as p
from pybullet_pinocchio_tests.robot_wrapper import RobotWrapper
from pybullet_pinocchio_tests.robot_control import RobotControl, ControlMethod
from scipy.spatial.transform import Rotation as Rotation
#
# import eigenpy
# eigenpy.switchToNumpyMatrix()

p.connect(p.GUI)
ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "three_revolute_planar", "three_revolute_planar.urdf")

TEST_FORWARD_KINEMATICS = False
TEST_JACOBIAN = False
TEST_JACOBIAN_DT = False

IMPEDANCE_CONTROL = True
JOINT_SPACE_CONTROL = False
TASK_SPACE_CONTROL = False


def get_link_name_idx_map(obj_id):
    link_idx_map = {p.getBodyInfo(obj_id)[0].decode('UTF-8'): -1, }
    for idx in range(p.getNumJoints(obj_id)):
        name = p.getJointInfo(obj_id, idx)[12].decode('UTF-8')
        link_idx_map[name] = idx
    return link_idx_map


def screw(x):
    assert len(x) == 3
    return np.array([[0.0, -x[2], x[1]],
                     [x[2], 0.0, -x[0]],
                     [-x[1], x[0], 0.0]])


def pitch_to_rotation(pitch):
    R = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                  [0.0, 1.0, 0.0],
                  [-math.sin(pitch), 0.0, math.cos(pitch)]])
    return R


class ManipulatorTest:
    def __init__(self):

        # Manipulator
        self.robot = p.loadURDF(URDF_PATH,
                                useFixedBase=True,
                                basePosition=[0.0, 0.0, 0.0],
                                flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE)
        self.a1 = 1.0
        self.a2 = 1.0
        self.a3 = 1.0
        self.dof = p.getNumJoints(self.robot)
        self.q = np.ndarray((3, ), dtype=float)
        self.v = np.ndarray((3, ), dtype=float)
        self.tau = np.ndarray((3,), dtype=float)

        print("The robot has {} dofs".format(self.dof))

        # Pinocchio based model
        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data = self.model.createData()
        self.ee_idx = self.model.getFrameId("ee")
        print("Ee link index is: {}".format(self.ee_idx))

        print("model name = {}".format(self.model.name))

        # Other vars
        self.timestep = 0.01  # 1/240.
        self.slowdown_factor = 1
        self.link_map = get_link_name_idx_map(self.robot)
        self.ee_idx_bullet = self.link_map["ee"]
        self.prev_angJ, self.prev_linJ = None, None
        self.prevp_angJ, self.prevp_linJ = None, None
        self.track_x, self.track_y, self.track_z = None, None, None

        # Task space control
        self.Kp = np.diag(np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]))
        self.Kd = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        self.initial_q = np.array([-math.pi/2.0, 30 * math.pi / 180., 30 * math.pi / 180.])
        self.initial_rotation = None
        self.initial_position = None
        self.prev_time = time.time()
        print(self.link_map)

        # try out new wrapper
        self.robot_wrapper = RobotWrapper()
        self.robot_wrapper.init_from_urdf(URDF_PATH)
        self.controller = RobotControl(self.robot_wrapper, "ee")
        self.controller.set_control_method(ControlMethod.TASK_SPACE_CONTROL)

    def init_sim(self):
        self.disable_motors()

        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.timestep)
        p.setRealTimeSimulation(False)

    def sim(self):
        self.prev_linJ, self.prev_angJ = self.jacobian_bullet()
        self.prevp_linJ, self.prevp_angJ = self.jacobian_pinocchio()
        p.resetJointState(self.robot, 0, self.initial_q[0])
        p.resetJointState(self.robot, 1, self.initial_q[1])
        p.resetJointState(self.robot, 2, self.initial_q[2])
        self.update_state()

        R, t, v, w = self.forward_kinematics_pinocchio()
        self.initial_position = t
        self.initial_rotation = R

        if TEST_FORWARD_KINEMATICS:
            while True:
                print("======== Forward kinematics (model) =========")
                R, t, v, w = self.forward_kinematics_model()
                self.draw_pose_arrow(R, t)
                print("R = ")
                print(R)
                print("t = ")
                print(t)
                print("v = ")
                print(v)
                print("w = ")
                print(w)

                print("========= Forward kinematics (bullet) =========")
                R, t, v, w = self.forward_kinematics_bullet()
                print("R = ")
                print(R)
                print("t = ")
                print(t)
                print("v = ")
                print(v)
                print("w = ")
                print(w)

                print("========= Forward kinematics (pinocchio) =========")
                R, t, v, w = self.forward_kinematics_pinocchio()
                print("R = ")
                print(R)
                print("t = ")
                print(t)
                print("v = ")
                print(v)
                print("w = ")
                print(w)
                self.advance()

        if TEST_JACOBIAN:
            while True:
                print("======== Jacobian (model) =========")
                linJ, angJ = self.jacobian_model()
                print("linJ = ")
                print(linJ)
                print("angJ = ")
                print(angJ)

                print("======== Jacobian (bullet) =========")
                linJ, angJ = self.jacobian_bullet()
                print("linJ = ")
                print(linJ)
                print("angJ = ")
                print(angJ)

                print("======== Jacobian (pinocchio) =========")
                linJ, angJ = self.jacobian_pinocchio()
                print("linJ = ")
                print(linJ)
                print("angJ = ")
                print(angJ)
                self.advance()

        if TEST_JACOBIAN_DT:
            while True:
                print("======== Jacobian time vatriation (model) =========")
                lindJ, angdJ = self.jacobian_dt_model()
                print("lindJ = ")
                print(lindJ)
                print("angdJ = ")
                print(angdJ)

                print("======== Jacobian time variation (bullet) =========")
                lindJ, angdJ = self.jacobian_dt_bullet()
                print("lindJ = ")
                print(lindJ)
                print("angdJ = ")
                print(angdJ)

                print("======== Jacobian time variation (pinocchio) =========")
                lindJ, angdJ = self.jacobian_dt_pinocchio()
                print("lindJ = ")
                print(lindJ)
                print("angdJ = ")
                print(angdJ)
                self.advance()

        if JOINT_SPACE_CONTROL:
            q0 = p.addUserDebugParameter("q0", -3.14, 3.14, self.q[0])
            q1 = p.addUserDebugParameter("q1", -3.14, 3.14, self.q[1])
            q2 = p.addUserDebugParameter("q2", -3.14, 3.14, self.q[2])
            q = np.array([0.0]*3)
            while True:
                q[0] = p.readUserDebugParameter(q0)
                q[1] = p.readUserDebugParameter(q1)
                q[2] = p.readUserDebugParameter(q2)

                R, t, _, _ = self.forward_kinematics_pinocchio()
                self.draw_pose_arrow(R, t)
                self.joint_space_control(desired_q=q)
                self.advance()

        if TASK_SPACE_CONTROL:
            local_world_aligned = False
            dxc = p.addUserDebugParameter("dx", 0.0, 0.5, 0.0)
            dzc = p.addUserDebugParameter("dz", -1.5, 1.5, 0.0)
            dpc = p.addUserDebugParameter("dp", -180.0, 180.0, 0.0)
            while True:
                dx = p.readUserDebugParameter(dxc)
                dz = p.readUserDebugParameter(dzc)
                dp = p.readUserDebugParameter(dpc) * math.pi / 180.
                target_position = self.initial_position + np.array([dx, 0.0, dz])
                target_rotation = self.initial_rotation.dot(pitch_to_rotation(dp))
                if local_world_aligned:
                    self.task_space_control(target_position, target_rotation)
                else:
                    self.task_space_control_wrapper(target_position, target_rotation)
                self.advance()

        if IMPEDANCE_CONTROL:
            dxc = p.addUserDebugParameter("dx", 0.0, 0.5, 0.0)
            dzc = p.addUserDebugParameter("dz", -1.5, 1.5, 0.0)
            dpc = p.addUserDebugParameter("dp", -180.0, 180.0, 0.0)

            while True:
                dx = p.readUserDebugParameter(dxc)
                dz = p.readUserDebugParameter(dzc)
                dp = p.readUserDebugParameter(dpc) * math.pi / 180.
                target_position = self.initial_position + np.array([dx, 0.0, dz])
                target_rotation = self.initial_rotation.dot(pitch_to_rotation(dp))
                self.impedance_control(target_position, target_rotation)
                self.advance()

    def advance(self):
        p.stepSimulation()
        # time.sleep(self.timestep * self.slowdown_factor)
        self.update_state()
        self.report_time()

    def report_time(self):
        # TODO do some proper reporting
        t = time.time()
        dt = t - self.prev_time
        self.prev_time = t

    def update_state(self):
        states = p.getJointStates(self.robot, range(0, 3))
        for idx, state in enumerate(states):
            self.q[idx] = state[0]
            self.v[idx] = state[1]
            self.tau[idx] = state[3]

    def disable_motors(self):
        """
        Disable the default active motors in each joint and set to zero the default linear and angular
        damping (both equal to 0.04)
        :return:
        """
        p.setJointMotorControlArray(self.robot, range(0, 3), p.VELOCITY_CONTROL, forces=[0.0]*3)
        for i in range(0, 3):
            p.changeDynamics(self.robot, i, linearDamping=0.04, angularDamping=0.04)

    #
    #    Forward kinematics
    #
    def forward_kinematics_model(self):
        # Pose
        c1, s1 = math.cos(self.q[0]), math.sin(self.q[0])
        c12, s12 = math.cos(self.q[0] + self.q[1]), math.sin(self.q[0] + self.q[1])
        c123, s123 = math.cos(self.q[0] + self.q[1] + self.q[2]), math.sin(self.q[0] + self.q[1] + self.q[2])

        x = self.a1 * c1 + self.a2 * c12 + self.a3 * c123
        y = 0.0
        z = -self.a1 * s1 - self.a2 * s12 - self.a3 * s123
        pitch = self.q[0] + self.q[1] + self.q[2]
        R = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                      [0.0, 1.0, 0.0],
                      [-math.sin(pitch), 0.0, math.cos(pitch)]])
        t = np.array([x, y, z])

        # Velocity
        v1 = self.v[0]
        v12 = self.v[0] + self.v[1]
        v123 = self.v[0] + self.v[1] + self.v[2]
        vx = -self.a1 * s1 * v1 - self.a2 * s12 * v12 - self.a3 * s123 * v123
        vy = 0.0
        vz = -self.a1 * c1 * v1 - self.a2 * c12 * v12 - self.a3 * c123 * v123
        v = np.array([vx, vy, vz])
        w = np.array([0.0, v123, 0.0])

        return R, t, v, w

    def forward_kinematics_pinocchio(self):
        pin.forwardKinematics(self.model, self.data, self.q, self.v)
        pin.updateFramePlacements(self.model, self.data)
        T = self.data.oMf[self.ee_idx]

        local_to_world_transform = pin.SE3(np.eye(3), np.array([0, 0, 0]))
        local_to_world_transform.rotation = T.rotation
        frame_placement = self.model.frames[self.ee_idx].placement
        parent_joint = self.model.frames[self.ee_idx].parent

        # y = frame_placement-1 * (v) --> converts joint velocity in body (LOCAL) frame to
        #                                 the frame velocity (in body frame)
        # frame_v = R_w_f * y         --> converts velocity from body (LOCAL) to world frame
        frame_v_local = frame_placement.actInv(self.data.v[parent_joint])
        frame_v = local_to_world_transform.act(frame_v_local)
        # print("Frame v [LOCAL] pin method = \n" + str(pin.getFrameVelocity(self.model, self.data, self.ee_idx)))
        # print("Frame v [LOCAL] by hand = \n" + str(frame_v_local))
        # print("Local to world transform = \n" + str(local_to_world_transform))
        # print("Linear transformed by hand = \n" + str(T.rotation.dot(frame_v_local.linear)))
        # print("Linear pin = \n" + str(frame_v.linear))
        # print("Angular transformed by hand = \n" + str(T.rotation.dot(frame_v_local.angular)))
        # print("Angular pin = \n" + str(frame_v.angular))

        return T.rotation, T.translation, frame_v.linear, frame_v.angular

    def forward_kinematics_bullet(self):
        state = p.getLinkState(self.robot, self.ee_idx_bullet, computeForwardKinematics=True,
                               computeLinkVelocity=True)
        t = np.array(state[4])
        R = Rotation.from_quat(state[5])
        R = R.as_dcm()
        v = np.array(state[6])
        w = np.array(state[7])
        return R, t, v, w

    #
    # Jacobians
    #
    def jacobian_model(self):
        c1, s1 = math.cos(self.q[0]), math.sin(self.q[0])
        c12, s12 = math.cos(self.q[0] + self.q[1]), math.sin(self.q[0] + self.q[1])
        c123, s123 = math.cos(self.q[0] + self.q[1] + self.q[2]), math.sin(self.q[0] + self.q[1] + self.q[2])
        linJ = np.array([[-self.a1 * s1 - self.a2 * s12 - self.a3 * s123, - self.a2 * s12 - self.a3 * s123, - self.a3 * s123],
                         [0.0, 0.0, 1.0],
                         [-self.a1 * c1 - self.a2 * c12 - self.a3 * c123, - self.a2 * c12 - self.a3 * c123, - self.a3 * c123]])
        angJ = np.array([[0, 0, 0],
                         [1, 1, 1],
                         [0, 0, 0]])
        return linJ, angJ

    def jacobian_bullet(self):
        # the jacobian is extracted from a call to the inverse dynamics which
        # requires the joint velocities and the desired joint accelerations
        # It makes sense therefore to provide 0 joint accelerations and velocities
        v = [0.0]*3
        a = [0.0]*3
        q = [0.0]*3
        for idx, val in enumerate(self.q):
            q[idx] = self.q[idx]
        linJ, angJ = p.calculateJacobian(self.robot, self.ee_idx_bullet, localPosition=[0.0, 0.0, 0.0],
                                         objPositions=q,
                                         objVelocities=v,
                                         objAccelerations=a)
        linJ = np.array([list(linJ[0]), list(linJ[1]), list(linJ[2])])
        angJ = np.array([list(angJ[0]), list(angJ[1]), list(angJ[2])])

        return linJ, angJ

    def jacobian_pinocchio(self):
        pin.framesForwardKinematics(self.model, self.data, self.q)
        pin.computeJointJacobians(self.model, self.data, self.q)
        J = pin.getFrameJacobian(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J2 = pin.getFrameJacobian(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL)
        R, _, _, _ = self.forward_kinematics_pinocchio()
        # print("Lin world jacobian from local = \n" + str(R.dot(J2[:3, :])))
        # print("Ang world jacobian from local = \n" + str(R.dot(J2[3:, :])))

        linJ = J[:3, :]
        angJ = J[3:, :]
        return linJ, angJ

    #
    # Jacobian time variation
    #
    def jacobian_dt_model(self):
        c1, s1 = math.cos(self.q[0]), math.sin(self.q[0])
        c12, s12 = math.cos(self.q[0] + self.q[1]), math.sin(self.q[0] + self.q[1])
        c123, s123 = math.cos(self.q[0] + self.q[1] + self.q[2]), math.sin(self.q[0] + self.q[1] + self.q[2])
        v1 = self.v[0]
        v12 = self.v[0] + self.v[1]
        v123 = self.v[0] + self.v[1] + self.v[2]
        lindJ = np.array([[-self.a1 * c1 * v1 - self.a2 * c12 * v12 - self.a3 * c123 * v123,
                           - self.a2 * c12 * v12 - self.a3 * c123 * v123,
                           - self.a3 * c123 * v123],
                          [0.0, 0.0, 0.0],
                          [self.a1 * s1 * v1 + self.a2 * s12 * v12 + self.a3 * s123 * v123,
                           self.a2 * s12 * v12 + self.a3 * s123 * v123,
                           self.a3 * s123 * v123]])
        angdJ = np.array([[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0]])
        return lindJ, angdJ

    def jacobian_dt_pinocchio(self):
        # with finite differences it works ok
        finite_differences = True
        if finite_differences:
            linJ, angJ = self.jacobian_pinocchio()
            lindJ = (linJ - self.prevp_linJ) / self.timestep
            angdJ = (angJ - self.prevp_angJ) / self.timestep
            self.prevp_linJ = linJ
            self.prevp_angJ = angJ
            return lindJ, angdJ

        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        pin.getFrameJacobian(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.v)
        dJ = pin.getFrameJacobianTimeVariation(self.model, self.data, self.ee_idx,
                                               pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # dJ_local = pin.getFrameJacobianTimeVariation(self.model, self.data, self.ee_idx,
        #                                        pin.ReferenceFrame.LOCAL)

        #     R, _, _, w = self.forward_kinematics_pinocchio()
        #     wx = screw(w)
        #
        #     J_local = pin.getFrameJacobian(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL)
        #     linJ_local = J_local[:3, :]
        #     lindJ_local = dJ_local[:3, :]
        #     lindJ = R.dot(lindJ_local - wx.dot(linJ_local))
        Jout = np.matrix(np.zeros((6, self.model.nv)), dtype=float)
        #
        # for i in range(3):
        #     T = self.data.oMf[self.ee_idx]
        #     vin = pin.Motion(dJ[:, i])
        #     vout = pin.Motion(dJ[:, i])
        #     vout.linear -= np.cross(T.translation, vin.angular, axis=0)
        #     Jout[:, i] = vout.vector

        lindJ = dJ[:3, :]
        angdJ = dJ[3:, :]
        return lindJ, angdJ

    def jacobian_dt_bullet(self):
        linJ, angJ = self.jacobian_bullet()
        lindJ = (linJ - self.prev_linJ)/self.timestep
        angdJ = (angJ - self.prev_angJ)/self.timestep
        self.prev_linJ = linJ
        self.prev_angJ = angJ
        return lindJ, angdJ

    def joint_space_control(self, desired_q):
        pin.computeAllTerms(self.model, self.data, self.q, np.array([0.0, 0.0, 0.0]))
        nl = self.data.nle
        print("dq= ")
        tau_pd = self.data.M.dot(np.array([10.0, 100.0, 100.0]) * (desired_q - self.q) +
                                 np.array([10.0, 5.0, 1.0]) * (-self.v))
        tau = nl + tau_pd
        print("Nle = " + str(nl))
        print("Tau pd = " + str(tau_pd))
        self.apply_motor_torques(tau)

    def impedance_control(self, target_position, target_rotation):
        pose_des = pin.SE3(target_rotation, target_position)

        R, t, v, w = self.forward_kinematics_pinocchio()
        pose_current = pin.SE3(R, t)

        # print(pose_current)
        # pose_target = pin.SE3.Interpolate(pose_current, pose_des, 0.01)
        # self.draw_pose_arrow(pose_des.rotation, pose_des.translation)

        # compute rotation error
        dR = Rotation.from_matrix(R.transpose().dot(target_rotation)).as_quat()
        rerr = 2*dR[3] * dR[:3]
        print("ree={}".format(rerr))
        dp = np.hstack([target_position - t, rerr])
        print("pose current: " + str(pose_current))
        print("pose desired: " + str(pose_des))

        print("dp=\n" + str(dp))
        # print("dp linear=\n" + str(dM.linear))
        # print("dp angular=\n" + str(dM.angular))
        dv = -pin.Motion(v, w).vector
        f = self.Kp.dot(dp) + self.Kd.dot(dv)

        pin.computeAllTerms(self.model, self.data, self.q, self.v)
        linJ, angJ = self.jacobian_pinocchio()
        J = np.vstack([linJ, angJ])
        y = J.transpose().dot(f)
        tau = y + self.data.nle

        # tau = pin.rnea(self.model, self.data, self.q, self.v, a)
        print("tau=" + str(tau))
        self.apply_motor_torques(tau)

    def task_space_control(self, target_position, target_rotation):
        pose_des = pin.SE3(target_rotation, target_position)

        R, t, v, w = self.forward_kinematics_pinocchio()
        pose_current = pin.SE3(R, t)

        # print(pose_current)
        # pose_target = pin.SE3.Interpolate(pose_current, pose_des, 0.01)
        # self.draw_pose_arrow(pose_des.rotation, pose_des.translation)
        dM = pin.log6(pose_current.actInv(pose_des))

        dp = dM.vector
        dp = np.array([0.0] * 6)
        dp = pose_current.actInv(pose_des)
        # compute rotation error
        dR = Rotation.from_matrix(R.transpose().dot(target_rotation)).as_quat()
        rerr = 2 * dR[3] * dR[:3]
        #print("ree={}".format(rerr))
        dp = np.hstack([target_position - t, rerr])

#        dp = np.hstack([target_position - t, np.array([0.0, 0.0, 0.0])])
        #print("pose current: " + str(pose_current))
        #print("pose desired: " + str(pose_des))

        #print("dp=\n" + str(dp))
        # print("dp linear=\n" + str(dM.linear))
        # print("dp angular=\n" + str(dM.angular))
        dv = -pin.Motion(v, w).vector
        xdd = self.Kp.dot(dp) + self.Kd.dot(dv)

        # Jacobians
        linJ, angJ = self.jacobian_pinocchio()
        lindJ, angdJ = self.jacobian_dt_pinocchio()
        J = np.vstack([linJ, angJ])
        dJ = np.vstack([lindJ, angdJ])

        # print("xdd=\n" + str(xdd))
        # print("v=\n" + str(self.v))
        # print("dJ=\n" + str(dJ))
        # print("J=\n" + str(J))
        # print("J+=\n" + str(np.linalg.pinv(J)))


        # print("========== " + str(len(a)))
        # print(a)
        #self.v = np.array([0.0, 0.0, 0.0])
        pin.computeAllTerms(self.model, self.data, self.q, self.v)
        y = np.linalg.pinv(J, rcond=0.09).dot(xdd - dJ.dot(self.v))
        tau = self.data.M.dot(y) + self.data.nle

        # tau = pin.rnea(self.model, self.data, self.q, self.v, a)
        #print("tau=" + str(tau))
        self.apply_motor_torques(tau)

    def task_space_control_local(self, target_position, target_rotation):
        pose_des = pin.SE3(target_rotation, target_position)

        R, t, v, w = self.forward_kinematics_pinocchio()
        pose_current = pin.SE3(R, t)

        #self.draw_pose_arrow(pose_des.rotation, pose_des.translation)

        # compute motion which moves current to target
        v_des = pin.log(pose_current.actInv(pose_des))
        twist = np.hstack([v, w])
        R = np.vstack([np.hstack([R, np.zeros((3, 3))]),
                       np.hstack([np.zeros((3, 3)), R])]
                      )

        v = R.transpose().dot(twist)
        v_err = v_des - v

        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.v)
        j = pin.getFrameJacobian(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL)
        dj = pin.getFrameJacobianTimeVariation(self.model, self.data, self.ee_idx, pin.ReferenceFrame.LOCAL)

        pin.computeAllTerms(self.model, self.data, self.q, self.v)

        y = np.linalg.pinv(j, rcond=0.001).dot(v_err - dj.dot(self.v))
        tau = self.data.M.dot(y) + self.data.nle

        self.apply_motor_torques(tau)

    def task_space_control_wrapper(self, target_position, target_orientation):
        target = pin.SE3(target_orientation, target_position)
        self.controller.set_task_target(target)
        tau = self.controller.advance(self.q, self.v)
        self.apply_motor_torques(tau)

    def apply_motor_torques(self, tau):
        assert len(tau) == 3
        p.setJointMotorControlArray(self.robot, range(0, 3), p.TORQUE_CONTROL, forces=tau)

    def draw_pose_arrow(self, R, t):
        xax = np.array([0.5, 0.0, 0.0])
        yax = np.array([0.0, 0.5, 0.0])
        zax = np.array([0.0, 0.0, 0.5])

        xax = R.dot(xax)
        yax = R.dot(yax)
        zax = R.dot(zax)

        tx = t + xax
        ty = t + yax
        tz = t + zax
        if self.track_x is None:
            self.track_x = p.addUserDebugLine(t, tx, [1, 0, 0], lineWidth=2, lifeTime=0.0)
            self.track_y = p.addUserDebugLine(t, ty, [0, 1, 0], lineWidth=2, lifeTime=0.0)
            self.track_z = p.addUserDebugLine(t, tz, [0, 0, 1], lineWidth=2, lifeTime=0.0)
        else:
            self.track_x = p.addUserDebugLine(t, tx, [1, 0, 0], lineWidth=2, replaceItemUniqueId=self.track_x)
            self.track_y = p.addUserDebugLine(t, ty, [0, 1, 0], lineWidth=2, replaceItemUniqueId=self.track_y)
            self.track_z = p.addUserDebugLine(t, tz, [0, 0, 1], lineWidth=2, replaceItemUniqueId=self.track_z)


if __name__ == "__main__":
    test = ManipulatorTest()
    test.init_sim()
    test.sim()
