import pybullet as p


class PyBulletSimulationBase:
    def __init__(self, time_step=None):
        p.connect(p.GUI)

        self.realtime_sim = True
        if not time_step:
            self.realtime_sim = False
        self.time_step = time_step

    def init_sim(self):
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(self.realtime_sim)

    def advance_simulation(self):
        if self.realtime_sim:
            p.stepSimulation()

    def advance(self):
        raise NotImplementedError("This method needs to be implemented for each simulation")
