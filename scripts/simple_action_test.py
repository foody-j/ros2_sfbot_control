#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SimpleActionTester(Node):
    def __init__(self):
        super().__init__('simple_action_tester')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # /joint_states의 실제 순서와 동일하게 맞춰야 합니다.
        self.joint_names = [
            'link1_1_joint', 'link2_1_joint', 'link3_1_joint',
            'link5_1_joint', 'link4_1_joint', 'link6_1_joint'
        ]

    def send_goal(self):
        # 목표: 2번 관절만 0.5 라디안 (약 28도) 움직이기
        goal_positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0] 
        duration = 4.0 # 4초 동안

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = goal_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        self.get_logger().info(f"Sending goal to move joint 2 to 0.5 rad...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Goal failed with error: {result.error_string} (Code: {result.error_code})')

def main(args=None):
    rclpy.init(args=args)
    action_tester = SimpleActionTester()
    action_tester.send_goal()
    action_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()