import pybullet as p


class JointCommandType:
    POSITION = p.POSITION_CONTROL
    VELOCITY = p.VELOCITY_CONTROL
    TORQUE = p.TORQUE_CONTROL


class JointCommand:
    def __init__(self):
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
    def __init__(self, names):
        self.commands = {name: JointCommand() for name in names}
