#
# In this short script, we show how to use RobotWrapper
# integrating different kinds of viewers
#
from copy import deepcopy
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer


class DummySimulation:
    def __init__(self, urdf_file, packages_dirs):
        composite_root = pin.JointModelComposite(3)
        composite_root.addJoint(pin.JointModelPX())
        composite_root.addJoint(pin.JointModelPY())
        composite_root.addJoint(pin.JointModelRZ())
        self.robot = RobotWrapper.BuildFromURDF(urdf_file, package_dirs=packages_dirs, root_joint=composite_root,
                                           verbose=True)
        self.model = self.robot.model
        self.data = self.robot.data
        self.viz = MeshcatVisualizer(self.model, self.robot.collision_model, self.robot.visual_model)

        self.robot.setVisualizer(self.viz)
        self.robot.initViewer()
        self.robot.loadViewerModel("pinocchio")

        self.q0 = self.robot.q0
        self.q = deepcopy(self.q0)
        self.u = np.zeros(self.model.nv)

        self.lb = self.model.lowerPositionLimit
        self.ub = self.model.upperPositionLimit

        # Set limits for the base
        self.lb[:3] = -np.ones(3)
        self.ub[:3] = np.ones(3)

        print(f"Robot loaded: nq={self.model.nq}, nv={self.model.nv}")

    def set_random(self):
        self.q = pin.randomConfiguration(self.model, self.lb, self.ub)

    def set_input(self, u):
        self.u = u

    def update(self):
        pin.forwardKinematics(self.model, self.data, self.q, self.u)
        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

    def get_velocity(self, frame_name):
        frame_idx = self.model.getFrameId(frame_name)
        v = pin.getFrameVelocity(self.model, self.data, frame_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return v

    def step(self, dt):
        self.q = pin.integrate(self.model, self.q, self.u * dt)
        self.robot.display(self.q)

