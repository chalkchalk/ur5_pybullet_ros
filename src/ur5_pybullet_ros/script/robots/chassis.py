import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
from ros_wrapper.ros_msg.ros_dtype import ROSDtype

ROS_CMD_VEL_TOPIC = "cmd_vel"

class Chassis:
    def __init__(self, ros_wrapper, robot_id, joint_id):
        self.ros_wrapper = ros_wrapper
        self.robot_id = robot_id
        self.joint_id = joint_id # x y theta chassis
        self.twist = np.array([0.0, 0.0, 0.0]) # x y theta
        self.pos = np.array([0.0, 0.0, 0.0])
        self.init_ros_wrapper()
    
    def init_ros_wrapper(self):
        self.ros_wrapper.add_subscriber(ROS_CMD_VEL_TOPIC, ROSDtype.TWIST, use_namespace=False, callback=self.cmdvel_callback)
    
    def cmdvel_callback(self, msg):
        self.twist[0] = msg.linear.x
        self.twist[1] = msg.linear.y
        self.twist[2] = msg.angular.z
        self.set_twist(self.twist)
        
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