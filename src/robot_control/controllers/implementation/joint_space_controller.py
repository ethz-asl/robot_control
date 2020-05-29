import numpy as np

from robot_control.modeling import RobotWrapper
from robot_control.controllers import RobotControllerBase


class JointSpaceController(RobotControllerBase):
    def __init__(self, robot, controlled_frame):
        super(JointSpaceController, self).__init__(robot)

        assert isinstance(robot, RobotWrapper)
        self.robot = robot
        self.controlled_frame = controlled_frame

        self.joint_target = self.robot.get_neutral_configuration()
        self.joint_target_set = False

        # Joint gains
        self.kp = np.diag([0] * self.robot.get_dof())
        self.kd = np.diag([0] * self.robot.get_dof())

    def set_kp(self, kp):
        self.kp = kp

    def set_kd(self, kd):
        self.kd = kd

    def set_task_target(self, joint_target):
        assert len(joint_target) == self.robot.get_dof()
        self.joint_target = joint_target
        self.joint_target_set = True

    def compute_command(self):
        if not self.joint_target_set:
            print("Joint target is not set yet")
            return

        q_err = self.joint_target - self.robot.get_q()
        v_err = - self.robot.get_v()

        q_dd_des = self.kp.dot(q_err) + self.kd.dot(v_err)
        tau = self.robot.get_inertia().dot(q_dd_des) + self.robot.get_nonlinear_terms()
        return tau
