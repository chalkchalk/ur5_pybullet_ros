import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryResult,FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectoryPoint




class JointTrajectoryActionServer:
    def __init__(self, joint_names, controller_name = "position_joint_trajectory_controller"):
        self.joint_names = joint_names
        self.goal = FollowJointTrajectoryGoal()
        self.joint_pos = []
        self.joint_vel = []
        self.server = actionlib.SimpleActionServer("/%s/follow_joint_trajectory"%(controller_name),
            FollowJointTrajectoryAction,execute_cb=self.execute_trajectory,
            auto_start=False
        )
        rospy.loginfo("server init")
        self.server.start()
    
    def execute_trajectory(self, goal):
        # goal = FollowJointTrajectoryGoal()
        rospy.loginfo("Received trajectory goal")
        print(goal)
        self = self.goal
        for i in range(10):
            if self.server.is_preempt_requested():
                rospy.loginfo("Trajectory execution preempted")
                self.server.set_preempted()
                return
            rospy.sleep(0.5)
            feedback = FollowJointTrajectoryFeedback()
            
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = goal.trajectory.joint_names
            feedback.actual = JointTrajectoryPoint()
            feedback.actual.positions = self.joint_pos
            feedback.actual.velocities = self.joint_vel
            feedback.actual.time_from_start = rospy.Duration(0.5 * i)
            self.server.publish_feedback(feedback)
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESS
        # 发布结果
        self.server.set_succeeded(result)
        rospy.loginfo("Trajectory execution completed")
    
    def update_current_state(self, pos, vel):
        self.joint_pos = pos
        self.joint_vel = vel

if __name__ == '__main__':
    rospy.init_node('position_joint_trajectory_controller_server')
    try:
        server = JointTrajectoryActionServer([], "ur5_controller")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
