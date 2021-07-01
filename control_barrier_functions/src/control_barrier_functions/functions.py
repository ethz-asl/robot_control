import numpy as np
import pinocchio as pin
from copy import deepcopy


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
    def __init__(self, model: pin.Model, data: pin.Data, frame1, frame2, D, P=None):
        super().__init__()
        self.model = deepcopy(model)
        self.data = deepcopy(data)
        self.frames = [frame1, frame2]
        self.frames_idxs = {frame : self.model.getFrameId(frame) for frame in self.frames}
        self.placements = {frame : np.zeros(3) for frame in self.frames}
        self.D = D
        if P is None:
            self.P = np.eye(3)
        else:
            self.P = P

    def update_data(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        self.placements = {frame : self.data.oMf[self.frames_idxs[frame]].translation for frame in self.frames}

    def get_linear_jacobian(self, frame_idx):
        return pin.getFrameJacobian(self.model, self.data, frame_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]

    def eval(self, x):
        self.update_data(x)
        delta = self.placements[self.frames[0]] - self.placements[self.frames[1]]
        return 0.5 * (delta.transpose().dot(self.P).dot(delta) - self.D ** 2).reshape(1,)

    def J(self, x):
        delta = self.placements[self.frames[0]] - self.placements[self.frames[1]]
        j0 = self.get_linear_jacobian(self.frames_idxs[self.frames[0]])
        j1 = self.get_linear_jacobian(self.frames_idxs[self.frames[1]])
        return delta.transpose().dot(self.P).dot(j0 - j1)


class CartesianReachLimitsFunction(CartesianLimitsFunction):
    def __init__(self, model: pin.Model, data: pin.Data, frame1, frame2, D, P=None):
        super().__init__(model, data, frame1, frame2, D, P)

    def eval(self, x):
        return - super().eval(x)

    def J(self, x):
        return - super().J(x)
