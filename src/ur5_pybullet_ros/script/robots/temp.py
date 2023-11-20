#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class PositionJointTrajectoryControllerServer:
    def __init__(self):
        rospy.init_node('position_joint_trajectory_controller_server')

        # 创建 Action Server
        self.server = actionlib.SimpleActionServer(
            'position_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory,
            auto_start=False
        )

        # 启动 Action Server
        self.server.start()

    def execute_trajectory(self, goal):
        rospy.loginfo("Received trajectory goal")

        # 在这里执行轨迹控制
        # 可以使用 goal.goal.trajectory 获取目标轨迹信息

        # 模拟一个简单的反馈
        for i in range(10):
            if self.server.is_preempt_requested():
                rospy.loginfo("Trajectory execution preempted")
                self.server.set_preempted()
                return

            # 模拟处理轨迹的过程
            rospy.sleep(0.5)

            # 创建一个简单的反馈
            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = goal.goal.trajectory.joint_names
            feedback.actual = JointTrajectoryPoint()
            feedback.actual.positions = [0.1 * i] * len(goal.goal.trajectory.joint_names)
            feedback.actual.velocities = [0.0] * len(goal.goal.trajectory.joint_names)
            feedback.actual.time_from_start = rospy.Duration(0.5 * i)

            # 发布反馈
            self.server.publish_feedback(feedback)

        # 模拟完成后的状态
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESS

        # 发布结果
        self.server.set_succeeded(result)
        rospy.loginfo("Trajectory execution completed")

if __name__ == '__main__':
    try:
        server = PositionJointTrajectoryControllerServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
