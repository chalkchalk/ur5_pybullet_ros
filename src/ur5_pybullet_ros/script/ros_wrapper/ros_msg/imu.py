from sensor_msgs.msg import Imu
from ros_wrapper.ros_msg.ros_msg_base import ROSMsgBase
import rospy
from typing import Type

class ImuData(ROSMsgBase):
    rosdtype = Imu
    def __init__(self, quat, ang_vel, lin_acc):
        super().__init__()
        self.imu = Imu()
        self.imu.orientation.x = quat[0]
        self.imu.orientation.y = quat[1]
        self.imu.orientation.z = quat[2]
        self.imu.orientation.w = quat[3]
        self.imu.angular_velocity.x = ang_vel[0]
        self.imu.angular_velocity.y = ang_vel[1]
        self.imu.angular_velocity.z = ang_vel[2]
        self.imu.linear_acceleration.x = lin_acc[0]
        self.imu.linear_acceleration.y = lin_acc[1]
        self.imu.linear_acceleration.z = lin_acc[2]
    
    @classmethod
    def transform_rosmsg(cls, data, ros_time, frame_id=""):
        ros_msg = data.imu
        ros_msg.header.frame_id = frame_id
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)
        return ros_msg