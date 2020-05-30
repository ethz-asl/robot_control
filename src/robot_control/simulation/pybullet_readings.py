class JointReading:
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0


class JointReadingVector:
    def __init__(self, names):
        self.readings = {name: JointReading() for name in names}
