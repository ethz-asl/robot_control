import os
import time
import numpy as np
from copy import deepcopy

from control_barrier_functions.simulation import DummySimulation
from control_barrier_functions.functions import JointLimitsFunction
from control_barrier_functions.solver import solve_qp, BarrierFunctionManager

DT = 0.01
ROOT = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = "/home/giuseppe/catkin_ws/src/robot_control/robot_control_assets/assets/arms/kinova3/kinova3.urdf"
PACKAGES_DIR = [""]

sim = DummySimulation(URDF_FILE, PACKAGES_DIR)
bf = BarrierFunctionManager()
bf.add(JointLimitsFunction(sim.lb, sim.ub))

u_des = np.ones(sim.model.nv) * 0.1

while True:
    sim.step(DT)

    # solve qp to avoid limits violation
    q = deepcopy(sim.q)
    u_opt = bf.solve(q, u_des)
    sim.set_input(u_opt)

    time.sleep(DT)
