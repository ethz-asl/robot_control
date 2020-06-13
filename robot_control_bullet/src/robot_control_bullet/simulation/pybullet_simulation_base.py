# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import pybullet as p


class PyBulletSimulationMode:
    GUI = p.GUI
    HEADLESS = p.DIRECT


class PyBulletSimulationBase:
    def __init__(self, time_step=None, mode=PyBulletSimulationMode.GUI):
        self.realtime_sim = False
        if not time_step:
            self.realtime_sim = True
        self.time_step = time_step
        self.mode = mode
        self.init_sim()

    def init_sim(self):
        p.connect(self.mode)
        p.setGravity(0, 0, -9.81)
        if not self.realtime_sim:
            p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(self.realtime_sim)

    def step_simulation(self):
        if not self.realtime_sim:
            p.stepSimulation()

    def run(self):
        raise NotImplementedError("This method needs to be implemented for each simulation")
