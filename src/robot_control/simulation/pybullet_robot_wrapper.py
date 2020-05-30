import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rotation

from robot_control.simulation import get_link_name_idx_map, get_link_idx_name_map
from robot_control.simulation import JointCommandVector, JointCommandType
from robot_control.simulation import JointReading, JointReadingVector

# TODO maybe refactoring and separate between joint mapping and link mapping


class PybulletRobotWrapper:
    def __init__(self, urdf_path, base_position, fixed_base=True):
        self.robot = p.loadURDF(urdf_path,
                                basePosition=base_position,
                                useFixedBase=fixed_base,
                                flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE)

        # init joint structure
        self.name_to_idx_map = get_link_name_idx_map(self.robot)
        self.idx_to_name_map = get_link_idx_name_map(self.robot)
        self.joint_commands = JointCommandVector(self.name_to_idx_map.keys())
        self.joint_readings = JointReadingVector(self.name_to_idx_map.keys())

        # state
        self.dof = len(self.name_to_idx_map)
        self.q = [0.0] * self.dof
        self.v = [0.0] * self.dof

        # track enabled ft sensors
        self.ft_sensors_enabled = []

    def get_readings(self):
        idxs = self.name_to_idx_map.values()
        js = p.getJointStates(self.robot, idxs)
        i = 0
        for j, idx in zip(js, idxs):
            name = self.idx_to_name_map[idx]
            jr = JointReading()
            jr.position = j[0]
            jr.velocity = j[1]
            self.q[i], self.v[i] = j[0], j[1]
            i += 1

            if name in self.ft_sensors_enabled:
                jr.reaction_forces = j[2]

            if self.joint_commands.commands[name].cmd_type == JointCommandType.TORQUE:
                jr.torque = self.joint_commands.commands[name].torque
            else:
                jr.torque = j[3]

    def set_commands(self, joint_commands):
        self.joint_commands = joint_commands

    def send_commands(self):
        for command in self.joint_commands.commands:
            p.setJointMotorControl2(self.robot,
                                    self.name_to_idx_map[command.name],
                                    command.cmd_type,
                                    targetPosition=command.position,
                                    targetVelocity=command.velocity,
                                    force=command.torque)

    def enable_ft_sensor(self, link_name):
        """ this actually enables the ft sensor at the corrsponding joint """
        p.enableJointForceTorqueSensor(self.robot, self.name_to_idx_map(link_name))

    def disable_motors(self):
        """
         Disable the default active motors in each joint and set to zero the default linear and angular
         damping (both equal to 0.04)
         :return:
        """
        idxs = self.name_to_idx_map.values()
        p.setJointMotorControlArray(self.robot, idxs, p.VELOCITY_CONTROL, forces=[0.0] * len(idxs))
        for idx in idxs:
            p.changeDynamics(self.robot, idx, linearDamping=0.1, angularDamping=0.1)

    def update_state(self):
        self.get_readings()
        return self.q, self.v

    def set_joint_state(self, q):
        for i, q in enumerate(q):
            p.resetJointState(self.robot, i + 1, q)

    def get_contact_wrench(self, object_id, link_name_A=-1, link_name_B=-1):
        link_idx_A = -1
        link_idx_B = -1
        if link_name_A != -1:
            link_idx_A = self.name_to_idx_map[link_name_A]
        if link_name_B != -1:
            link_idx_B = get_link_name_idx_map(object_id)[link_name_B]

        points = p.getContactPoints(self.robot, object_id, link_idx_A, link_idx_B)
        normal_vector_world = np.zeros(3)
        for point in points:
            normal_force = point[9]
            normal_direction = np.array(point[7])
            normal_vector_world += normal_force * normal_direction
        return normal_vector_world

    def get_link_pose_vel(self, name):
        """ Compute the forward kinematic relying on bullet """
        state = p.getLinkState(self.robot, self.name_to_idx_map[name], computeForwardKinematics=True,
                               computeLinkVelocity=True)
        t = np.array(state[4])
        R = Rotation.from_quat(state[5])
        R = R.as_matrix()
        v = np.array(state[6])
        w = np.array(state[7])
        return R, t, v, w