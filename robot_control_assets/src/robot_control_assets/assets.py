import os
import rospkg

ros_pack = rospkg.RosPack()
PACKAGE_PATH = ros_pack.get_path("robot_control_assets")


class RobotControlAssets:
    base_path = os.path.join(PACKAGE_PATH, "assets")

    def __init__(self):
        pass

    @classmethod
    def get_kinova_urdf_path(cls):
        return cls.create_path("arms", "kinova3", "kinova3.urdf")

    @classmethod
    def get_kinova_robotiq_gripper_urdf_path(cls):
        return cls.create_path("composite", "kinova3_gripper", "kinova3_gripper.urdf")

    @classmethod
    def get_panda_urdf_path(cls):
        return cls.create_path("arms", "panda", "panda.urdf")

    @classmethod
    def create_path(cls, *args):
        path = cls.base_path
        for arg in args:
            path = os.path.join(path, arg)
        return path
