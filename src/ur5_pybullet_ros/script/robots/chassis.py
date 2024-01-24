import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

class Chassis:
    def __init__(self, robot_id, joint_id):
        self.robot_id = robot_id
        self.joint_id = joint_id # x y theta chassis
        self.twist = np.array([0.0, 0.0, 0.0]) # x y theta
        self.pos = np.array([0.0, 0.0, 0.0])
    
    def set_twist(self, twist):
        self.update_pos()
        self.twist = twist
        theta = self.pos[2]
        twist_w = np.array([0.0, 0.0, 0.0])
        twist_w[0] = twist[0] * np.cos(theta) - twist[1] * np.sin(theta)
        twist_w[1] = twist[0] * np.sin(theta) + twist[1] * np.cos(theta)
        twist_w[2] = twist[2]
        for i in range(3):
            p.setJointMotorControl2(self.robot_id, self.joint_id[i] , p.VELOCITY_CONTROL, targetVelocity=twist_w[i],force=1000 )
    
    def update_pos(self):
        chassis_state = p.getLinkState(self.robot_id, self.joint_id[3])
        self.pos[0] = chassis_state[0][0]
        self.pos[1] = chassis_state[0][1]
        self.pos[2] = Rotation.from_quat(chassis_state[1]).as_euler('xyz')[2]