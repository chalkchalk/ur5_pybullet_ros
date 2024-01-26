from robots.robot_base import RobotBase
import os
import pybullet as p
import gin
import math
from ros_wrapper.ros_wrapper import RosWrapper
from ros_wrapper.ros_msg.ros_dtype import ROSDtype
from ros_wrapper.ros_msg.robot_joint_state import RobotJointState
from ros_wrapper.joint_trajectory_action_server import  JointTrajectoryActionServer, ActionState
from controller.trajectory import get_trajectory_from_ros_msg
from controller.trajectory_follower import TrajecyFollower, FollowState
from camera.camera import Camera
from robots.chassis import Chassis
from lidar.lidar import Lidar
import time
import threading

ROS_SET_ANGLE_TOPIC = "set_angle"
ROS_JOINT_STATES_TOPIC = "joint_states"
ROS_JOINT_ANGLE_TOPIC = "joint_angles"


@gin.configurable
class UR5(RobotBase):
    def __init__(self, urdf_file, base_pos, base_ori, inital_angle, gripper_range, arm_joint, eef_joint, chassis_joint, lidar_joint):
        self.name = "UR5"
        urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/" + urdf_file
        super().__init__(self.name, urdf_file, base_pos, base_ori, inital_angle, gripper_range, arm_joint, eef_joint)
        self.reset()
        self.time = 0
        self.set_angle = [inital_angle] # we use list to make it a mutable variable, so the callback of ros can change this value naturely
        self.init_ros_interface()
        self.chassis = Chassis(self.ros_wrapper, self.id, self.get_joint_id(chassis_joint))
        self.lidar = Lidar(self.ros_wrapper, self.id, self.get_joint_id([lidar_joint])[0])
        self.camera = Camera(self.ros_wrapper, self.id, self.get_joint_id([eef_joint])[0])
        self.trajectory_follower = TrajecyFollower(self. arm_joint)
        self.joint_info_all = {}
        self.joint_arm_info = {}
        self.joint_info_all = self.get_rotate_joint_info_all()
        self.joint_arm_info = self.get_joint_obs()
        self.ros_pub_thread = threading.Thread(
            target=self.pub_ros_info_thread)
        self.ros_pub_thread.setDaemon(True)
        self.ros_pub_thread.start()

    def init_ros_interface(self):
        self.ros_wrapper = RosWrapper("ur5_pybullet")
        self.joint_tra_action_server = JointTrajectoryActionServer(self. arm_joint, "ur5_controller")
        self.ros_wrapper.add_subscriber(ROS_SET_ANGLE_TOPIC, ROSDtype.FLOAT_ARRAY, self.set_angle)
        self.ros_wrapper.add_publisher(ROS_JOINT_STATES_TOPIC, ROSDtype.JOINT_STATE, False)
        self.ros_wrapper.add_publisher(ROS_JOINT_ANGLE_TOPIC, ROSDtype.FLOAT_ARRAY)
        

    def post_control(self):
        # self.pub_ros_info()
        # end_pos, end_orn = self.get_end_state()
        # self.camera.update_pose(end_pos, end_orn)
        self.publish_sensor()
        pass

    def pre_control(self):
        self.time = self.ros_wrapper.ros_time
        self.update_sensor()
        self.joint_info_all = self.get_rotate_joint_info_all()
        self.joint_arm_info = self.get_joint_obs()
        self.trajectory_follower.loop(self.joint_arm_info["positions"], self.joint_arm_info ["velocities"])
        if self.joint_tra_action_server.new_goal:
            # self.set_angle[0] = self.joint_tra_action_server.goal.trajectory.points[-1].positions
            trajectory = get_trajectory_from_ros_msg(self.joint_tra_action_server.goal, self.time)
            self.trajectory_follower.set_trajectory(trajectory)
            self.joint_tra_action_server.new_goal = False

        if self.trajectory_follower.state == FollowState.RUNNNING:
            sef_point = self.trajectory_follower.get_control_point(self.time)
            self.set_angle[0] = sef_point.positions
        
    def pub_ros_info(self):
        self.joint_tra_action_server .update_current_state(self.joint_arm_info ["positions"], self.joint_arm_info ["velocities"])
        joint_state = RobotJointState(self.rotate_joint_names, self.joint_info_all["positions"], self.joint_info_all["velocities"], self.joint_info_all["torques"])
        self.ros_wrapper.publish_msg(ROS_JOINT_STATES_TOPIC, joint_state)
        # if self.camera.bgr is not None:
        #     self.ros_wrapper.publish_msg(ROS_IMAGE_TOPIC, self.camera.bgr)
        # if self.camera.point_cloud is not None:
        #     self.ros_wrapper.publish_msg(ROS_POINT_CLOUD_TOPIC, self.camera.point_cloud, "camera_link")
        
        # print(joint_info)
    
    def pub_ros_info_thread(self):
        last_time = 0
        while True:
            if last_time != self.ros_wrapper.ros_time:
                self.pub_ros_info()
            last_time = self.ros_wrapper.ros_time
            time.sleep(0.01)
    
    def move_gripper(self, open_length):
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation
    
    def set_base_twist(self, twist):
        self.chassis.set_twist(twist)
    
    def update_sensor(self):
        # self.lidar.update_rays()
        pass

    def publish_sensor(self):
        self.chassis.publish_odom()
        pass

CONFIG_FILE = (os.path.dirname(os.path.abspath(__file__)) + "/../config/ur5_default.gin")
gin.parse_config_file(CONFIG_FILE)

if __name__ == "__main__":
    p.connect(p.GUI)
    ur5_robot = UR5()
    while True:
        pass