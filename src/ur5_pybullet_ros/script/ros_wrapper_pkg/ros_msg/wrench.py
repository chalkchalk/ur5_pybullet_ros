from geometry_msgs.msg import WrenchStamped
from ros_wrapper_pkg.ros_msg.ros_msg_base import ROSMsgBase
import numpy as np
import rospy

class Wrench(ROSMsgBase):
    def __init__(self): # just us np.array or list
        self.rosdtype = WrenchStamped
    
    @classmethod
    def transform_rosmsg(cls, data, ros_time, frame_id=""):
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = data[3]
        ros_msg.wrench.torque.y = data[4]
        ros_msg.wrench.torque.z = data[5]
        ros_msg.header.frame_id = frame_id
        ros_msg.header.stamp = rospy.Time.from_sec(ros_time)
        return ros_msg
    
    