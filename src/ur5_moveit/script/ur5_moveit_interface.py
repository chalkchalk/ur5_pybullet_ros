import sys
import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    
    
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class UR5MoveitInterface:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5_move_group_python_interface")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)
        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)
        
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        
        self.goal_pose_pub = rospy.Publisher('/goal_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    
    def go_to_joint_state(self, joint_state):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        assert len(joint_state) == len(joint_goal), "joint_state must have same dim with joint_goal!"
        joint_goal = joint_state
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position, orientation): 
        """
        @param: position       position x y z 
        @param: orientation     x y z w
        @returns: bool
        """
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.pose.orientation.x = orientation[0]
        pose_goal.pose.orientation.y = orientation[1]
        pose_goal.pose.orientation.z = orientation[2]
        pose_goal.pose.orientation.w = orientation[3]
        
        pose_goal.pose.position.x = position[0]
        pose_goal.pose.position.y = position[1]
        pose_goal.pose.position.z = position[2]
        
        
        self.goal_pose_pub.publish(pose_goal)

        move_group.set_pose_target(pose_goal.pose)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal.pose, current_pose, 0.01)

if __name__ == "__main__":
    ur5_moveit_interface = UR5MoveitInterface()
    # ur5_moveit_interface.go_to_joint_state([0,-0.5,0,0,0,0])
    ur5_moveit_interface.go_to_pose_goal([0.4,0.3,0.8], [0,0,0,1])