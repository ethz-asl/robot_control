import os
import time
import numpy as np
from copy import deepcopy

from control_barrier_functions.simulation import DummySimulation
from control_barrier_functions.functions import *
from control_barrier_functions.solver import solve_qp, BarrierFunctionManager

DT = 0.01
ROOT = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = "/home/giuseppe/catkin_ws/src/robot_control/robot_control_assets/assets/arms/kinova3/kinova3.urdf"
PACKAGES_DIR = [""]

sim = DummySimulation(URDF_FILE, PACKAGES_DIR)

# Joint limits function
jlf = JointLimitsFunction(sim.lb, sim.ub)

# End Effector reach function
eerf = CartesianReachLimitsFunction(model=sim.model, data=sim.data,
                                    frame1="base_link", frame2="end_effector_link", D=0.5,
                                    P=np.diag(np.array([1, 1, 0])))

scf = CartesianLimitsFunction(model=sim.model, data=sim.data,
                              frame1="half_arm_2_link", frame2="end_effector_link", D=0.15,
                              P=np.diag(np.array([1, 1, 1])))

# Join barrier functions
bf = BarrierFunctionManager()
bf.add(jlf)
bf.add(eerf)
bf.add(scf)

# u_des = np.ones(sim.model.nv) * 0.1
u_des = np.zeros(sim.model.nv)
u_des[4] = 0.1
u_des[6] = -0.1
u_des[8] = -0.1

while True:
    sim.step(DT)

    # solve qp to avoid limits violation
    q = deepcopy(sim.q)
    # print(f"u_in={u_des.transpose()}")
    u_opt = bf.solve(q, u_des)
    # print(f"u_out={u_opt.transpose()}")

    sim.set_input(u_opt)
    time.sleep(DT)
