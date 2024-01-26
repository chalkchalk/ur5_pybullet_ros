from enum import Enum
from ros_wrapper.ros_msg.float_array import FloatArray
from ros_wrapper.ros_msg.imu import ImuData
from ros_wrapper.ros_msg.point_cloud import PointCloud
from ros_wrapper.ros_msg.robot_joint_state import RobotJointState
from ros_wrapper.ros_msg.ros_clock import ROSClock
from ros_wrapper.ros_msg.float import Float
from ros_wrapper.ros_msg.force import Force
from ros_wrapper.ros_msg.wrench import Wrench
from ros_wrapper.ros_msg.image import Image

class ROSDtype(Enum):
    FLOAT = Float
    FLOAT_ARRAY = FloatArray
    JOINT_STATE = RobotJointState
    WRENCH = Wrench
    FORCE = Force
    IMU = ImuData
    CLOCK  = ROSClock
    IMAGE = Image
    POINT_CLOUD = PointCloud