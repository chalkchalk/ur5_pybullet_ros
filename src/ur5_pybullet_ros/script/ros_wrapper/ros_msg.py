import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Imu, Image, PointCloud2, PointField
import sensor_msgs.point_cloud2
from geometry_msgs.msg import WrenchStamped
from enum import Enum
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from cv_bridge import CvBridge


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

class ROSDtype(Enum):
    FLOAT = Float64
    FLOAT_ARRAY = Float64MultiArray
    JOINT_STATE = JointState
    WRENCH = WrenchStamped
    FORCE = WrenchStamped
    IMU = Imu
    CLOCK  = Clock
    IMAGE = Image
    POINT_CLOUD = PointCloud2
    
class ROSClock(Clock):
    def __init__(self, time):
        super().__init__()
        self.clock = rospy.Time.from_sec(time)
        # header=Header(stamp=rospy.Time.from_sec(time))
        # self = Clock()

class RobotJointState(JointState):
    def __init__(self, name, pos, vel, tor):
        super().__init__()
        self.name = name
        self.position = pos
        self.velocity = vel
        self.effort = tor

class ImuData(Imu):
    def __init__(self, quat, ang_vel, lin_acc):
        super().__init__()
        self.orientation.x = quat[0]
        self.orientation.y = quat[1]
        self.orientation.z = quat[2]
        self.orientation.w = quat[3]
        self.angular_velocity.x = ang_vel[0]
        self.angular_velocity.y = ang_vel[1]
        self.angular_velocity.z = ang_vel[2]
        self.linear_acceleration.x = lin_acc[0]
        self.linear_acceleration.y = lin_acc[1]
        self.linear_acceleration.z = lin_acc[2]

def data_to_ros_msg(data, dtype:ROSDtype, ros_time, frame_id=""):
    ros_msg = None
    if dtype == ROSDtype.FLOAT_ARRAY:
        data = np.array(data)
        ros_msg = Float64MultiArray()
        ros_msg.data = data

    elif dtype == ROSDtype.JOINT_STATE:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.FORCE:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = 0
        ros_msg.wrench.torque.y = 0
        ros_msg.wrench.torque.z = 0
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.WRENCH:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = data[3]
        ros_msg.wrench.torque.y = data[4]
        ros_msg.wrench.torque.z = data[5]
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)

    elif dtype == ROSDtype.IMU:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)
    
    elif dtype == ROSDtype.CLOCK:
        ros_msg = data
    
    elif dtype == ROSDtype.IMAGE:
        bridge = CvBridge()
        ros_msg = bridge.cv2_to_imgmsg(data,  encoding="passthrough")
    
    elif dtype == ROSDtype.POINT_CLOUD:
        header = Header()
        header.stamp = rospy.Time.from_sec(ros_time)
        header.frame_id = frame_id
        open3d_cloud = data
        points=np.asarray(open3d_cloud.points)
        if True: # not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=points
        else: # XYZ + RGB
            fields=FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
            colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
            cloud_data=np.c_[points, colors]
        ros_msg = sensor_msgs.point_cloud2.create_cloud(header, fields, cloud_data)
    return ros_msg