import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from robot_control.robot_wrapper import RobotWrapper
from robot_control.controllers.robot_controller_base import RobotControllerBase


class ImpedanceController(RobotControllerBase):
    def __init__(self, robot, controlled_frame):
        super(ImpedanceController, self).__init__(robot)

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
        # Not working yet
        # TODO (giuseppe) switch to end effector dynamics
        if not self.task_target_set:
            print("Task target is not set yet")
            return

        pose_des = self.task_target
        pose_current = self.robot.get_frame_placement(self.controlled_frame)

        # compute rotation error
        dR = Rotation.from_matrix(pose_current.rotation.transpose().dot(pose_des.rotation)).as_quat()
        rotation_err = 2 * dR[3] * dR[:3]
        dp = np.hstack([pose_des.translation - pose_current.translation, rotation_err])

        v = self.robot.get_frame_velocity(self.controlled_frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        dv = -v.vector
        f = self.kp.dot(dp) + self.kd.dot(dv)

        self.robot.compute_all_terms()
        j = self.robot.get_frame_jacobian(self.controlled_frame, ref=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        y = j.transpose().dot(f)
        tau = y + self.robot.get_nonlinear_terms()
        return tau

