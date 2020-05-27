import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from robot_control.robot_wrapper import RobotWrapper
from robot_control.controllers.robot_controller_base import RobotControllerBase


class OperationalSpaceController(RobotControllerBase):
    def __init__(self, robot, controlled_frame):
        super(OperationalSpaceController, self).__init__(robot)
        self.controller_name = "op_space_controller"
        assert isinstance(robot, RobotWrapper)
        self.robot = robot
        self.controlled_frame = controlled_frame

        # Target is assumed to be in a world frame
        self.task_target = pin.SE3(np.eye(3), np.zeros((3, 1)))
        self.task_target_set = False

        # Operational space gains
        self.kp = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.kd = np.diag([10.0] * 6)

        # store for re-usage
        self.j = None
        self.dj = None

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
        dR = Rotation.from_matrix(pose_current.rotation.T.dot(pose_des.rotation)).as_quat()
        rotation_err = 2 * dR[3] * dR[:3]
        position_err = pose_current.rotation.T.dot(pose_des.translation - pose_current.translation)
        p_err = np.hstack([position_err, rotation_err])

        # velocity error (local frame)
        v_des = pin.log(pose_current.actInv(pose_des)).vector
        v_current = self.robot.get_frame_velocity(self.controlled_frame, pin.ReferenceFrame.LOCAL)
        v_err = v_des - v_current

        j, dj = self.robot.get_all_frame_jacobians(self.controlled_frame)
        self.j = j
        self.dj = dj
        self.robot.compute_all_terms()

        # end effector inertia matrix
        qd = self.robot.get_v()
        M_inv = self.robot.get_inertia_inverse()
        M_ee_inv = np.dot(j, np.dot(M_inv, j.T))

        # ! increasing the condition number causes the controller not to work
        # TODO (giuseppe) investigate this further
        M_ee = np.linalg.pinv(M_ee_inv, rcond=0.00001)

        w_d = self.kp.dot(p_err) + self.kd.dot(v_err)
        tau_acc = j.T.dot(M_ee.dot(w_d - dj.dot(qd)))
        tau = tau_acc + self.robot.get_nonlinear_terms()
        return tau

