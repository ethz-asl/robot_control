import time
import numpy as np
from scipy.spatial.transform import Rotation


def get_nullspace(a):
    """ Compute the nullspace of a matrix"""
    n = a.shape[1]
    return np.eye(n) - np.linalg.pinv(a, rcond=0.06).dot(a)


def get_rotation_error(r_desired, r_current):
    qd = Rotation.from_matrix(r_desired).as_quat()
    qe = Rotation.from_matrix(r_current).as_quat()

    eta_d, eta_e = qd[3], qe[3]
    eps_d, eps_e = qd[:3], qe[:3]
    err = eta_e * eps_d - eta_d * eps_e - np.cross(eps_d, eps_e)
    return err


class TrajectoryGenerator:
    def __init__(self):
        self.t = 0
        self.start_time = time.time()
        self.radius = 0.10

    def reset(self):
        self.t = 0
        self.start_time = time.time()

    def set_radius(self, r):
        self.radius = r

    def get_radius(self):
        return self.radius

    def advance_circular_trajectory(self, max_angle=None):
        if self.start_time:
            self.t = time.time() - self.start_time

        orn = [0, np.pi, 0.0]
        v = 0.2
        if self.t*v > max_angle:
            self.t = max_angle/v
        pos = [self.radius * np.cos(v*self.t), self.radius * np.sin(v*self.t), 0.0]
        orn[2] += v*self.t
        orn = Rotation.from_euler('xyz', orn).as_quat()
        return pos, orn


if __name__ == "__main__":
    r1 = Rotation.from_euler('xyz', [0, 0, 180]).as_matrix()
    r2 = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
    print("Rotation error = {}".format(get_rotation_error(r1, r2)))

    r1 = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
    r2 = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
    print("Rotation error = {}".format(get_rotation_error(r1, r2)))

