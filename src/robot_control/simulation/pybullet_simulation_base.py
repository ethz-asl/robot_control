# Created by Giuseppe Rizzi
# email grizzi@mavt.ehtz.ch

import pybullet as p


class PyBulletSimulationBase:
    def __init__(self, time_step=None):
        self.realtime_sim = False
        if not time_step:
            self.realtime_sim = True
        self.time_step = time_step
        self.init_sim()

    def init_sim(self):
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(self.realtime_sim)

    def step_simulation(self):
        if not self.realtime_sim:
            p.stepSimulation()

    def run(self):
        raise NotImplementedError("This method needs to be implemented for each simulation")
