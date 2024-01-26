import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import gin
import os
import time
import threading

@gin.configurable
class Lidar:
    def __init__(self, robot_id, frame_id, num_of_rays, min_range, max_range, debug_beams):
        self.robot_id = robot_id
        self.frame_id = frame_id
        self.num_of_rays = num_of_rays
        self.min_range = min_range
        self.max_range = max_range
        self.angle_increment = 2.0 * np.pi / num_of_rays
        self.ray_from_init, self.ray_to_init = self.init_rays()
        print(self.ray_to_init)
        self.link_state = p.getLinkState(self.robot_id, frame_id)
        self.debug_beams = debug_beams
        self.rays_data = np.zeros(num_of_rays)
        self.update_rays_thread = threading.Thread(
            target=self.update_rays_thread_fun)
        self.update_rays_thread.setDaemon(True)
        self.update_rays_thread.start()
        

    def init_rays(self):
        ray_from = np.zeros((self.num_of_rays, 3))
        ray_to = np.zeros((self.num_of_rays, 3))
        for i in range(self.num_of_rays):
            theta = self.angle_increment * i
            ray_from[i,:] = np.array([self.min_range * np.cos(theta), self.min_range * np.sin(theta), 0.0])
            ray_to[i,:] = np.array([self.max_range * np.cos(theta), self.max_range * np.sin(theta), 0.0])
        return ray_from, ray_to
    
    def update_rays(self):
        if self.debug_beams:
            p.removeAllUserDebugItems()
        self.link_state = p.getLinkState(self.robot_id, self.frame_id)
        ray_from, ray_to = self.get_rays(self.link_state[0], self.link_state[1])
        results = p.rayTestBatch(ray_from, ray_to, 4)
        if results:
            for i in range(self.num_of_rays):
                dist = results[i][2] * (self.max_range - self.min_range)
                self.rays_data[i] = dist
                if self.debug_beams:
                    hitObjectUid = results[i][0]
                    if hitObjectUid < 0:
                        # draw a line on pybullet gui for debug purposes in green because it did not hit any obstacle
                        p.addUserDebugLine(ray_from[i], ray_to[i], [1, 0, 0])
                    else:
                        # draw a line on pybullet gui for debug purposes in red because it hited obstacle, results[i][3] -> hitPosition
                        p.addUserDebugLine(ray_from[i], results[i][3], [0, 1, 0])
    
    def get_ray_data(self):
        return self.rays_data
    
    def update_rays_thread_fun(self):
        while True:
            self.update_rays()
            time.sleep(0.04)

    def get_rays(self, position, orientation):
        ray_from = self.ray_from_init.copy()
        ray_to = self.ray_to_init.copy()
        rot_mat = Rotation.from_quat(orientation).as_matrix() # p.getMatrixFromQuaternion(orientation)
        ray_from = np.dot(rot_mat, ray_from.transpose()).transpose() + np.array(position)
        ray_to = np.dot(rot_mat, ray_to.transpose()).transpose() + np.array(position)
        return ray_from, ray_to
        
        
        

CONFIG_FILE = os.path.dirname(os.path.abspath(__file__)) + "/../config/lidar.gin"
gin.parse_config_file(CONFIG_FILE)
        
        