import pybullet as p
from robots.ur5 import UR5
import gin
import time
import pybullet_data

@gin.configurable
class Environment():
    def __init__(self, setRealTimeSimulation):
        self.client = p.connect(p.GUI)
        self.robot = UR5()
        p.setRealTimeSimulation(setRealTimeSimulation)
        p.setGravity(0, 0, -9.81)
        # p.resetDebugVisualizerCamera(1.674, 70, -50.8, [0, 0, 0])+
        self.load_scenes()

    def step(self, action):
        self.robot.move_ee(action, "joint")
        p.stepSimulation()
        time.sleep(0.001)

    def load_scenes(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf', [0, 0, -0.63], [0, 0, 0, 1])

CONFIG_FILE = ("config/env_default.gin")
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    env = Environment()
    while True:
        env.step(env.robot.arm_rest_poses)