class RobotControllerBase:
    def __init__(self, robot):
        self.robot = robot
        self.controller_name = ""

    def get_name(self):
        return self.controller_name

    def compute_command(self):
        raise NotImplementedError("This method needs to be implemented.")

    def advance(self, q, v):
        self.robot.update_state(q, v)
        return self.compute_command()
