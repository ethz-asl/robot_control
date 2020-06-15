# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

from robot_control_bullet.simulation import get_actuated_joints_info


class JointReading:
    def __init__(self):
        self.dof = 1
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.reaction_forces = []


class JointReadingVector():
    def __init__(self, obj_id):
        self.readings = {}
        joints_info = get_actuated_joints_info(obj_id)
        for joint in joints_info.values():
            reading = JointReading()
            reading.dof = joint.dof
            if reading.dof != 1:
                reading.position = [0.0] * joint.dof
                reading.velocity = [0.0] * joint.dof
                reading.torque = [0.0] * joint.dof
            self.readings[joint.name] = reading
