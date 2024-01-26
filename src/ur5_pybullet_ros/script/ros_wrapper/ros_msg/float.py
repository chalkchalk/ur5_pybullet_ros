from std_msgs.msg import Float64
from ros_wrapper.ros_msg.ros_msg_base import ROSMsgBase

class Float(ROSMsgBase):
    rosdtype = Float64
    def __init__(self): # just us double
        pass
    
    @classmethod
    def transform_rosmsg(cls, data, ros_time, frame_id=""):
        ros_msg = Float64
        ros_msg.data = data
        return ros_msg
    
    