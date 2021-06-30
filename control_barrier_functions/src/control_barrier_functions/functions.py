import numpy as np


class ZeroingBarrierFunction:
    def __init__(self, priority=0):
        self.priority = priority

    def eval(self, x: np.array):
        raise NotImplementedError()

    def J(self, x: np.array):
        raise NotImplementedError()


class JointLimitsFunction(ZeroingBarrierFunction):
    def __init__(self, q_min: np.array, q_max: np.array):
        super().__init__()
        self.q_min = q_min
        self.q_max = q_max

    def eval(self, q: np.array):
        return (self.q_max - q) * (q - self.q_min) / (self.q_max - self.q_min)

    def J(self, q):
        return np.diag(self.q_max - self.q_min - 2 * q)


class CartesianLimitsFunction(ZeroingBarrierFunction):
    def __init__(self, frame1, frame2, D):
        super().__init__()
        self.frame1 = frame1
        self.frame2 = frame2
        self.D = D

    def eval(self, x):
        pass

    def J(self, x, jacobians):
        pass

