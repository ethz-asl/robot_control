# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import pybullet as p
from robot_control_bullet.simulation import get_actuated_joints_info


class JointCommandType:
    POSITION = p.POSITION_CONTROL
    VELOCITY = p.VELOCITY_CONTROL
    TORQUE = p.TORQUE_CONTROL


class JointCommand:
    def __init__(self):
        self.dof = 1
        self.cmd_type = JointCommandType.POSITION
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0

    def set_command_type(self, cmd_type):
        self.cmd_type = cmd_type

    def set_position(self, pos):
        self.position = pos

    def set_velocity(self, vel):
        self.velocity = vel

    def set_torque(self, tau):
        self.torque = tau


class JointCommandVector:
    def __init__(self, obj_id):
        self.commands = {}
        joints_info = get_actuated_joints_info(obj_id)
        for joint in joints_info.values():
            cmd = JointCommand()
            cmd.dof = joint.dof
            if joint.dof != 1:
                cmd.position = [0.0] * joint.dof
                cmd.velocity = [0.0] * joint.dof
                cmd.torque = [0.0] * joint.dof
            self.commands[joint.name] = cmd
