import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rotation

from collections import OrderedDict, namedtuple


def get_link_name_idx_map(obj_id):
    """
    Return a map containing link name : link index
    """
    link_idx_map = OrderedDict({p.getBodyInfo(obj_id)[0].decode('UTF-8'): -1, })
    for idx in range(p.getNumJoints(obj_id)):
        if idx != -1:
            name = p.getJointInfo(obj_id, idx)[12].decode('UTF-8')
            link_idx_map[name] = idx
    return link_idx_map


def get_link_idx_name_map(obj_id):
    """
    Return a map containing link idx : link name
    """
    link_name_map = OrderedDict({p.getBodyInfo(obj_id)[0].decode('UTF-8'): -1, })
    for idx in range(p.getNumJoints(obj_id)):
        if idx != -1:
            name = p.getJointInfo(obj_id, idx)[12].decode('UTF-8')
            link_name_map[idx] = name
    return link_name_map


def get_actuated_joints_info(obj_id):
    """ Return joints info in terms of named tuples """
    JointInfo = namedtuple('JointInfo', ['name', 'idx', 'joint_type', 'dof'])
    joints_info = {}
    for idx in range(p.getNumJoints(obj_id)):
        info = p.getJointInfo(obj_id, idx)
        name = info[1].decode('UTF-8')
        idx = info[0]
        joint_type = info[2]
        if joint_type != p.JOINT_FIXED:
            dof = 1
            if joint_type == p.JOINT_PLANAR:
                dof = 2
            if joint_type == p.JOINT_SPHERICAL:
                dof = 3
            joint_info = JointInfo(name=name, idx=idx, joint_type=joint_type, dof=dof)
            joints_info[name] = joint_info
    return joints_info


class DebugPose:
    """ Add user debug parameter to input a pose"""
    def __init__(self):
        self.xc = p.addUserDebugParameter("x", -1.0, 1.0, 0.0)
        self.yc = p.addUserDebugParameter("y", -1.0, 1.0, 0.0)
        self.zc = p.addUserDebugParameter("z", -1.0, 1.0, 0.0)
        self.rollc = p.addUserDebugParameter("roll", -180.0, 180.0, 0.0)
        self.pitchc = p.addUserDebugParameter("pitch", -180.0, 180.0, 0.0)
        self.yawc = p.addUserDebugParameter("yaw", -180.0, 180.0, 0.0)

    def get_debug_pose(self):
        x = p.readUserDebugParameter(self.xc)
        y = p.readUserDebugParameter(self.yc)
        z = p.readUserDebugParameter(self.zc)
        roll = p.readUserDebugParameter(self.rollc)
        pitch = p.readUserDebugParameter(self.pitchc)
        yaw = p.readUserDebugParameter(self.yawc)
        return np.array([x, y, z]), Rotation.from_euler(seq='xyz', angles=[roll, pitch, yaw], degrees=True).as_matrix()


class DebugWrench:
    """ Add user debug parameter to input a wrench"""
    def __init__(self):
        self.Fxc = p.addUserDebugParameter("Fx", -1.0, 1.0, 0.0)
        self.Fyc = p.addUserDebugParameter("Fy", -1.0, 1.0, 0.0)
        self.Fzc = p.addUserDebugParameter("Fz", -1.0, 1.0, 0.0)
        self.Mxc = p.addUserDebugParameter("Mx", -0.1, 0.1, 0.0)
        self.Myc = p.addUserDebugParameter("My", -0.1, 0.1, 0.0)
        self.Mzc = p.addUserDebugParameter("Mz", -0.1, 0.1, 0.0)

    def get_debug_wrench(self):
        Fx = p.readUserDebugParameter(self.Fxc)
        Fy = p.readUserDebugParameter(self.Fyc)
        Fz = p.readUserDebugParameter(self.Fzc)
        Mx = p.readUserDebugParameter(self.Mxc)
        My = p.readUserDebugParameter(self.Myc)
        Mz = p.readUserDebugParameter(self.Mzc)
        return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])
