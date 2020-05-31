import numpy as np
import pinocchio as pin


class RobotWrapper:
    """
    This is the main class wrapping pinocchio for modeling the robot.
    It provides access to all kino-dynamic quantities such as frames/joints placements
    inertia, jacobians and so on.
    Furthermore, pinocchio additionally exposes algorithms for forward and inverse dynamics and their
    derivatives. The modeling is based on the spatial formalism introduced by Featherstone.
    Check https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5569032 for a beginner introduction.
    All vectors belong to three main families, Motion, Forces and SE3 for 6d velocities, forces and poses respectively.
    """
    def __init__(self):
        self.model = pin.Model()
        self.data = pin.Data(self.model)
        self.q = np.array([])
        self.v = np.array([])
        self.tau = np.array([])
        self.nq = 0
        self.nv = 0

    def init_from_urdf(self, urdf_path):
        """
        Initialize the model and data structures
        :return:
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.v = np.array([0.0] * self.model.nv)
        self.tau = np.array([0.0] * self.model.nv)
        self.nq = self.model.nq
        self.nv = self.model.nv

    def get_dof(self):
        """
        :return: the dofs of the robot which is not the same as the length of the vector q but it is the length of the
        vector v (the tangent configuration space). q could be longer. As an example, continuous joints are parametrized
        by the cos and sin of the angle and 2 entries of q account for each of them.
        Currently this is not supported, meaning that we expect all joints to be revolute!!!
        """
        return self.nv

    def get_neutral_configuration(self):
        """
        :return: the configuration associated with zero gravity torques.
        """
        return pin.neutral(self.model)

    def get_random_configuration(self):
        """
        :return: a random random configuration q
        """
        return pin.randomConfiguration(self.model)

    def update_state(self, q, v, update_kinematics=True):
        """
        Set the internal robot state which is the generalized positions and velocities.
        :return:
        """
        self.q = np.asarray(q)
        self.v = np.asarray(v)
        if update_kinematics:
            self.forward_kinematics()

    def get_q(self):
        return self.q

    def get_v(self):
        return self.v

    def forward_kinematics(self):
        """
        Compute the forward kinematics (joint placements and velocities) and update also the corresponding
        attached frames (TODO probably some function calls are redundant).
        :return:
        """
        pin.forwardKinematics(self.model, self.data, self.q, self.v)
        pin.updateFramePlacements(self.model, self.data)

    def get_frame_placement(self, name):
        """
        Return the frame placement in the world coordinate frame.
        :param name: the frame name.
        :return: (SE3) frame placement.
        """
        frame_id = self.frame_id(name)
        if frame_id:
            return self.data.oMf[frame_id]

    def get_frame_velocity(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Get a frame velocity in the reference frame expressed by ref.
        :param name: the frame's name.
        :param ref: the reference frame, an instance type pinocchio.ReferenceFrame.
        :return: the frame velocity as a pinocchio.Motion object.
        """
        frame_id = self.frame_id(name)
        if frame_id:
            return pin.getFrameVelocity(self.model, self.data, frame_id, ref)

    def get_joint_jacobian(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Get a joint jacobian in the reference frame expressed by ref.
        :param name: the joint's name.
        :param ref: the reference frame, an instance type pinocchio.ReferenceFrame.
        :return: the joint jacobian of type numpy.array().
        """
        joint_id = self.model.getJointId(name) if self.model.existJointName(self.model, name) else None
        if joint_id:
            self.forward_kinematics()
            return pin.getJointJacobian(self.model, self.data, joint_id, ref)

    def get_joint_jacobian_dt(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Get a joint jacobian time variation velocity in the reference frame expressed by ref.
        :param name: the frame's name.
        :param ref: the reference frame, an instance type pinocchio.ReferenceFrame.
        :return: the joint jacobian time variation of type numpy.array().
        """
        joint_id = self.frame_id(name)
        if joint_id:
            pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.v)
            return pin.getJointJacobianTimeVariation(self.model, self.data, joint_id, ref)

    def get_frame_jacobian(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Get a frame jacobian in the reference frame expressed by ref.
        :param name: the frame's name.
        :param ref: the reference frame, an instance type pinocchio.ReferenceFrame.
        :return: the frame jacobian of type numpy.array().
        """
        frame_id = self.frame_id(name)
        if frame_id:
            pin.computeJointJacobians(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            return pin.getFrameJacobian(self.model, self.data, frame_id, ref)

    def get_frame_jacobian_dt(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Get a frame jacobian time variation in the reference frame expressed by ref.
        :param name: the frame's name.
        :param ref: the reference frame, an instance type pinocchio.ReferenceFrame.
        :return: the frame jacobian time variation of type numpy.array().
        """
        if ref == pin.ReferenceFrame.LOCAL_WORLD_ALIGNED:
            print("LOCAL_WORLD_ALIGNED is not supported")
            return

        frame_id = self.frame_id(name)
        if frame_id:
            pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.v)
            pin.updateFramePlacements(self.model, self.data)
            return pin.getFrameJacobianTimeVariation(self.model, self.data, frame_id, ref)

    def get_all_frame_jacobians(self, name, ref=pin.ReferenceFrame.LOCAL):
        """
        Do not call the previous function to avoid unnecessary twice call to updateFramePlacement.
        :param name: the name of the query frame.
        :param ref: the reference frame for the jacobian (LOCAL or WORLD).
        :return: the jacobian and its time derivative.
        """
        if ref == pin.ReferenceFrame.LOCAL_WORLD_ALIGNED:
            print("LOCAL_WORLD_ALIGNED is not supported")
            return

        frame_id = self.frame_id(name)
        if frame_id:
            pin.computeJointJacobians(self.model, self.data, self.q)
            pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.v)
            pin.updateFramePlacements(self.model, self.data)
            J = pin.getFrameJacobian(self.model, self.data, frame_id, ref)
            dJ = pin.getFrameJacobianTimeVariation(self.model, self.data, frame_id, ref)
            return J, dJ

    def get_inertia(self):
        """
        :return: the inertia matrix.
        """
        return self.data.M

    def get_inertia_inverse(self):
        """
        :return: the inverse of the inertia matrix
        """
        pin.computeMinverse(self.model, self.data, self.q)
        return self.data.Minv

    def get_nonlinear_terms(self):
        """
        :return: the non linear effects given by the gravity and coriolis terms.
        """
        return self.data.nle

    def compute_all_terms(self):
        """
        Compute all the terms M, non linear effects and Jacobians in the same loop.
        """
        pin.computeAllTerms(self.model, self.data, self.q, self.v)

    def frame_id(self, name):
        return self.model.getFrameId(name) if self.model.existFrame(name) else None

    def joint_id(self, name):
        return self.model.getJointId(name) if self.model.existJointName(name) else None

    def print_info(self):
        print("=== Frames info === ")
        for frame in self.model.frames:
            print(frame)
        print("=================== ")
        print("=== Joints info === ")
        for joint in self.model.joints:
            print(joint)
