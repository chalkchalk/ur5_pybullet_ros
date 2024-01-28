#!/usr/bin/env python3
import pybullet as p
from robots.ur5 import UR5
import gin
import numpy as np
import time
import pybullet_data
from ros_wrapper.ros_msg.ros_dtype import ROSDtype
import os
from scipy.spatial.transform import Rotation

ROS_CLOCK_TOPIC = "clock"

@gin.configurable
class Environment():
    def __init__(self, setRealTimeSimulation, dt, realtime_factor):
        self.client = p.connect(p.GUI) # p.connect(p.DIRECT)
        p.setRealTimeSimulation(setRealTimeSimulation)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(dt)
        self.robot = UR5()
        self.dt = dt
        self.time = 0.0
        self.realtime_factor = realtime_factor
        self.ros_wrapper = self.robot.ros_wrapper
        self.ros_wrapper.add_publisher(ROS_CLOCK_TOPIC, ROSDtype.CLOCK, False)
        self.udrf_path = os.path.dirname(os.path.abspath(__file__)) + "/urdf/objects/"
        self.load_scenes()

    def step(self):
        start_time = time.time()   
        self.time += self.dt
        self.ros_wrapper.ros_time = self.time
        self.ros_wrapper.publish_msg(ROS_CLOCK_TOPIC, ROSDtype.CLOCK.value(self.time))
        action = self.robot.set_angle[0]    
        self.robot.apply_control(action, "joint")
        # self.robot.set_base_twist([0.0,0.0, 0.3])
        p.stepSimulation()
        end_time = time.time()
        elapsed_time = end_time - start_time
        if self.dt / self.realtime_factor - elapsed_time > 0:
            time.sleep(self.dt / self.realtime_factor - elapsed_time)

    def load_scenes(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # print(pybullet_data.getDataPath()) #/usr/local/lib/python3.8/dist-packages/pybullet_data
        p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
        table1 = p.loadURDF(self.udrf_path + 'table/table.urdf', [0.8, -1.5, -0.2], [0, 0, 0, 1], useFixedBase=True)
        cube_small = p.loadURDF(self.udrf_path + 'cube/cube.urdf', [0.8, -1.6, 0.6], [0, 0, 0, 1], globalScaling = 0.2)
        table2 = p.loadURDF(self.udrf_path + 'table/table.urdf', [-6.5, 1.5, -0.2], [0, 0, 0, 1], useFixedBase=True)
        
        self.load_room()

    def load_room(self):
        p.loadURDF(self.udrf_path + 'block10.urdf', [-3, 2.5, 0.5], [0, 0, 0, 1], useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block10.urdf', [-3, -2.5, 0.5], [0, 0, 0, 1], useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block5.urdf', [-8, 0.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block5.urdf', [2, 0.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block2.urdf', [-1, 1.5, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        p.loadURDF(self.udrf_path + 'block3.urdf', [-3, -1.0, 0.5], Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat(), useFixedBase=True)
        static_cube1 = p.loadURDF(self.udrf_path + 'cube/cube.urdf', [-3, -2, 0.3], [0, 0, 0, 1], globalScaling = 0.7, useFixedBase=True)
        
        
        
CONFIG_FILE = os.path.dirname(os.path.abspath(__file__)) + "/config/env_default.gin"
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    env = Environment()
    while True:
        env.step()