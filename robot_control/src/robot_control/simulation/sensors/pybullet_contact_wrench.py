# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import numpy as np
import pybullet as p

import pinocchio as pin
from scipy.spatial.transform import Rotation


def get_contact_wrench_b_on_a(object_a, object_b, reference_link_idx):
    _, _, _, _, w_t_r, R_w_r = p.getLinkState(object_a, reference_link_idx, computeForwardKinematics=True)
    R_w_r = Rotation.from_quat(R_w_r).as_matrix()
    w_f_r_tot = pin.Force(np.zeros(6))
    points = p.getContactPoints(object_a, object_b)

    for point in points:
        # get tf from contact point to reference link
        w_t_i = np.array(point[6])
        w_t_ri = w_t_i - w_t_r  # vector going from r to i
        tf_r_i = pin.SE3(np.eye(3), w_t_ri)

        # get contact force (only the normal)
        normal_force = point[9]
        normal_direction = np.array(point[7])
        w_f_i = pin.Force(np.hstack([normal_force * normal_direction, np.zeros(3)]))

        # transport to wrench in reference link world oriented frame
        w_f_r = tf_r_i.act(w_f_i)
        w_f_r_tot += w_f_r

    # Rotate wrench in reference link frame
    O = np.zeros((3, 3))
    R_r_w_diag = np.vstack([np.hstack([R_w_r.T, O]), np.hstack([O, R_w_r.T])])
    r_f_r = R_r_w_diag.dot(w_f_r_tot.vector)
    return r_f_r
