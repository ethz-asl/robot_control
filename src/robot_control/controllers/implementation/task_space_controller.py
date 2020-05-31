# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from robot_control.modeling import RobotWrapper
from robot_control.controllers import RobotControllerBase
from robot_control.controllers.utilities import get_nullspace


class TaskSpaceController(RobotControllerBase):
    def __init__(self, robot, controlled_frame):
        super(TaskSpaceController, self).__init__(robot)

        assert isinstance(robot, RobotWrapper)
        self.robot = robot
        self.controlled_frame = controlled_frame

        # Target is assumed to be in a world frame
        self.task_target = pin.SE3(np.eye(3), np.zeros((3, 1)))
        self.task_target_set = False

        # Task space gains
        self.kp = np.diag([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        self.kd = np.diag([10, 10, 10, 10, 10, 10])

        # null space variables
        self.kqd_ns = np.diag([0.01] * self.robot.get_dof())
        self.kqp_res = np.diag([0.01] * self.robot.get_dof())
        self.q_rest = self.robot.get_neutral_configuration()

    def set_kp(self, kp):
        self.kp = kp

    def set_kd(self, kd):
        self.kd = kd

    def set_task_target(self, task_target):
        assert isinstance(task_target, pin.SE3)
        self.task_target = task_target
        self.task_target_set = True

    def compute_command(self):
        pose_current = self.robot.get_frame_placement(self.controlled_frame)
        pose_des = self.task_target
        if not self.task_target_set:
            pose_des = pose_current

        # pose error (local frame)
        dR = Rotation.from_matrix(pose_current.rotation.transpose().dot(pose_des.rotation)).as_quat()
        rotation_err = 2 * dR[3] * dR[:3]
        position_err = pose_current.rotation.transpose().dot(pose_des.translation - pose_current.translation)
        p_err = np.hstack([position_err, rotation_err])

        # velocity error (local frame)
        v_des = pin.log(pose_current.actInv(pose_des)).vector
        v_current = self.robot.get_frame_velocity(self.controlled_frame, pin.ReferenceFrame.LOCAL)
        v_err = v_des - v_current

        j, dj = self.robot.get_all_frame_jacobians(self.controlled_frame)
        self.robot.compute_all_terms()

        y = np.linalg.pinv(j, rcond=0.01).dot(self.kp.dot(p_err) + self.kd.dot(v_err) - dj.dot(self.robot.get_v()))
        eps = -self.kqd_ns.dot(self.robot.get_v()) - self.kqp_res.dot(self.robot.get_q() - self.q_rest)
        tau_null_space = get_nullspace(j).dot(eps)
        tau = self.robot.get_inertia().dot(y) + self.robot.get_nonlinear_terms() + tau_null_space
        return tau



