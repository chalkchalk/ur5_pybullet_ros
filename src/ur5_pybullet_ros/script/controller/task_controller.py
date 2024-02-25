#!/usr/bin/env python3
from controller.ur5_moveit_interface import UR5MoveitInterface
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import time
from enum import Enum
from std_msgs.msg import Float64



class JointConfiguration(Enum):
    UPRIGHT = [0.0,-0.5 * np.pi,0,0,0,0]
    DOWN_VIEW = [0.0, -0.5 *np.pi, 0.45 * np.pi, -0.5 * np.pi, -0.5 * np.pi, 0]

class TaskController:
    def __init__(self):
        self.moveit_interface = UR5MoveitInterface()
        self.moveit_interface.move_group.set_pose_reference_frame("base_link")
        self.movebase_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.grip_ratio_pub = rospy.Publisher("/ur5_pybullet/gripper_open_ratio", Float64, queue_size=1)
        
        self.navi_target = [np.array([0, 0, 0]), 0.0]
        self.robot_pos = [np.array([0, 0, 0]), 0.0]
        self.work1_pos = [np.array([0.72, -0.82, 0.0]), -0.5 * np.pi]
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pos[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y ,msg.pose.pose.position.z]
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.robot_pos[1] = Rotation.from_quat(orientation).as_euler('xyz')[2]
        
    def send_navigation_target(self, pos):
        position = pos[0]
        yaw = pos[1]
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        orientation = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False).as_quat()
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.movebase_goal_pub.publish(msg)
    
    def set_arm_joint_configuration(self, config:JointConfiguration):
        self.moveit_interface.go_to_joint_state(config.value)
    
    def set_gripper_ratio(self, ratio):
        msg = Float64()
        msg.data = ratio
        self.grip_ratio_pub.publish(msg)
    
    def step(self):
        task_controller.send_navigation_target(self.work1_pos)
    
    
        


if __name__ == "__main__":
    rospy.init_node("task_controller")
    task_controller = TaskController()
    task_controller.set_arm_joint_configuration(JointConfiguration.DOWN_VIEW)
    while not rospy.is_shutdown():
        task_controller.step()
        time.sleep(0.1)
    
    