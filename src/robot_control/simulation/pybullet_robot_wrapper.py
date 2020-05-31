import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rotation

from robot_control.simulation import get_link_name_idx_map, get_link_idx_name_map, get_actuated_joints_info
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
        self.actuated_joints_info = get_actuated_joints_info(self.robot)
        self.dof = sum([joint.dof for joint in self.actuated_joints_info.values()])

        self.joint_commands = JointCommandVector(self.robot)
        self.joint_readings = JointReadingVector(self.robot)

        # state
        self.q = [0.0] * self.dof
        self.v = [0.0] * self.dof

        # track enabled ft sensors
        self.ft_sensors_enabled = []

    def get_readings(self):
        state_idx = 0
        for joint in self.actuated_joints_info.values():
            joint_reading = JointReading()
            if joint.dof == 1:
                joint_state = p.getJointState(self.robot, joint.idx)
                joint_reading.position = joint_state[0]
                joint_reading.velocity = joint_state[1]
                joint_reading.torque = joint_state[3]
                self.q[state_idx], self.v[state_idx] = joint_state[0], joint_state[1]
                state_idx += 1
            else:
                joint_state = p.getJointStateMultiDof(self.robot, joint.idx)
                joint_reading.position = joint_state[0]
                joint_reading.velocity = joint_state[1]
                joint_reading.torque = joint_state[3]
                for q, v in zip(joint_state[0], joint_state[1]):
                    self.q[state_idx], self.v[state_idx] = q, v
                    state_idx += 1

            if joint.name in self.ft_sensors_enabled:
                joint_reading.reaction_forces = joint_state[2]

            if self.joint_commands.commands[joint.name].cmd_type == JointCommandType.TORQUE:
                joint_reading.torque = self.joint_commands.commands[joint.name].torque

    def set_command(self, joint_name, command_type, value):
        self.joint_commands.commands[joint_name].set_command_type(command_type)
        if command_type == JointCommandType.POSITION:
            self.joint_commands.commands[joint_name].set_position(value)
        elif command_type == JointCommandType.VELOCITY:
            self.joint_commands.commands[joint_name].set_velocity(value)
        elif command_type == JointCommandType.TORQUE:
            self.joint_commands.commands[joint_name].set_torque(value)

    def send_commands(self):
        for joint_name, command in self.joint_commands.commands.items():
            if command.dof == 1:
                if command.cmd_type != JointCommandType.TORQUE:
                    torque = 100 # high limit TODO (giuseppe) this should be the actuator limit
                else:
                    torque = command.torque
                p.setJointMotorControl2(self.robot,
                                        self.actuated_joints_info[joint_name].idx,
                                        command.cmd_type,
                                        targetPosition=command.position,
                                        targetVelocity=command.velocity,
                                        force=torque)
            else:
                p.setJointMotorControlMultiDof(self.robot,
                                               self.actuated_joints_info[joint_name].idx,
                                               targetPosition=command.position,
                                               targetVelocity=command.velocity,
                                               force=torque)

    def enable_ft_sensor(self, link_name):
        """ this actually enables the ft sensor at the corresponding joint """
        p.enableJointForceTorqueSensor(self.robot, self.name_to_idx_map[link_name])

    def disable_motors(self):
        """
         Disable the default active motors in each joint and set to zero the default linear and angular
         damping (both equal to 0.04)
         :return:
        """
        for joint in self.actuated_joints_info.values():
            p.setJointMotorControl2(self.robot, joint.idx, p.VELOCITY_CONTROL, force=0.0)
            p.changeDynamics(self.robot, joint.idx, linearDamping=0.1, angularDamping=0.1)

    def update_state(self):
        self.get_readings()
        return self.q, self.v

    def set_joint_state(self, q):
        for i, q in enumerate(q):
            p.resetJointState(self.robot, i + 1, q)

    def get_contact_wrench(self, object_in_contact, link_name_A=-1, link_name_B=-1):
        link_idx_A = -1
        link_idx_B = -1
        if link_name_A != -1:
            link_idx_A = self.name_to_idx_map[link_name_A]
        if link_name_B != -1:
            link_idx_B = get_link_name_idx_map(object_in_contact)[link_name_B]

        points = p.getContactPoints(self.robot, object_in_contact, link_idx_A, link_idx_B)
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

    def __str__(self):
        info = "Link: idx = {}\n".format(self.name_to_idx_map)
        for idx in range(p.getNumJoints(self.robot)):
            info += "Joint {} : {}\n".format(idx, p.getJointInfo(self.robot, idx))
        return info
