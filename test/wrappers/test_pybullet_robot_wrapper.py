import os
import math
import time
import pybullet as p
from robot_control.simulation import PybulletRobotWrapper, JointCommandType

ASSETS_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "assets",)
URDF_PATH = os.path.join(ASSETS_PATH, "arms", "kinova3", "kinova3.urdf")

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
robot_sim = PybulletRobotWrapper(URDF_PATH, [0.0, 0.0, 0.0], fixed_base=True)
robot_sim.set_joint_state([0., math.pi/8, 0.0, math.pi/2, 0, math.pi/3, -math.pi/2])
robot_sim.disable_motors()
robot_sim.enable_ft_sensor("end_effector_link")
step_counter = 0

while True:
    q, v = robot_sim.update_state()
    robot_sim.set_command("joint_1", JointCommandType.POSITION, 0.0)
    robot_sim.set_command("joint_2", JointCommandType.POSITION, math.pi/8)
    robot_sim.set_command("joint_3", JointCommandType.POSITION, 0.0)
    robot_sim.set_command("joint_4", JointCommandType.POSITION, math.pi/2)
    robot_sim.set_command("joint_5", JointCommandType.POSITION, 0.0)
    robot_sim.set_command("joint_6", JointCommandType.POSITION, math.pi/3)
    robot_sim.set_command("joint_7", JointCommandType.VELOCITY, 1.0)
    robot_sim.send_commands()

    p.stepSimulation()
    time.sleep(1/240.)

    step_counter += 1
    if (step_counter % 100) == 0:
        print(" === ")
        print("q = {}\nv = {}".format(q, v))
