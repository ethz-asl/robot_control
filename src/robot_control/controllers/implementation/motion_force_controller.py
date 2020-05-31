# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from robot_control.modeling import RobotWrapper
from robot_control.controllers import RobotControllerBase


class MotionForceController(RobotControllerBase):
    def __init__(self, robot, controlled_frame):
        super(MotionForceController, self).__init__(robot)

        assert isinstance(robot, RobotWrapper)
        self.robot = robot
        self.controlled_frame = controlled_frame
        self.controller_name = "motion_force_controller"

        # Target is assumed to be in a world frame
        self.task_target = pin.SE3(np.eye(3), np.zeros((3, 1)))
        self.task_target_set = False

        # Operational space gains
        self.kp = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.kd = np.diag([10.0] * 6)

        self.Sm = np.eye(6)                 # motion selection matrix
        self.Sf = np.zeros((6, 6))          # force selection matrix
        self.f = np.zeros(6)                # desired target wrench

    def set_kp(self, kp):
        self.kp = kp

    def set_kd(self, kd):
        self.kd = kd

    def set_Sm(self, Sm):
        self.Sm = Sm

    def set_Sf(self, Sf):
        self.Sf = Sf

    def set_wrench_target(self, f):
        assert len(f) == 6
        self.f = f

    def get_target_wrench(self):
        return self.f

    def set_task_target(self, task_target):
        assert isinstance(task_target, pin.SE3)
        self.task_target = task_target
        self.task_target_set = True

    def compute_command(self):
        if not self.task_target_set:
            print("Task target is not set yet")
            return

        pose_des = self.task_target
        pose_current = self.robot.get_frame_placement(self.controlled_frame)

        # pose error (local frame)
        dR = Rotation.from_matrix(pose_current.rotation.T.dot(pose_des.rotation)).as_quat()
        rotation_err = 2 * dR[3] * dR[:3]
        position_err = pose_current.rotation.T.dot(pose_des.translation - pose_current.translation)
        p_err = np.hstack([position_err, rotation_err])

        # velocity error (local frame)
        v_des = pin.log(pose_current.actInv(pose_des)).vector
        v_current = self.robot.get_frame_velocity(self.controlled_frame, pin.ReferenceFrame.LOCAL)
        v_err = v_des - v_current

        j, dj = self.robot.get_all_frame_jacobians(self.controlled_frame)
        self.robot.compute_all_terms()

        # end effector inertia matrix
        qd = self.robot.get_v()
        M_inv = self.robot.get_inertia_inverse()
        M_ee_inv = np.dot(j, np.dot(M_inv, j.T))

        # ! increasing the condition number causes the controller not to work
        # TODO (giuseppe) investigate this further
        M_ee = np.linalg.pinv(M_ee_inv, rcond=0.00001)

        w_d = self.Sm.dot(self.kp.dot(p_err) + self.kd.dot(v_err))
        f = self.Sf.dot(self.f)
        tau_acc = j.T.dot(M_ee.dot(w_d - dj.dot(qd)) + f)
        tau = tau_acc + self.robot.get_nonlinear_terms()

        return tau
