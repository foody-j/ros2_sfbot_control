#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
import sys
import numpy as np
import subprocess
import tempfile
import os

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    print("Required Python packages not installed. Please run: pip install roboticstoolbox-python spatialmath-python matplotlib")
    sys.exit(1)

def load_robot_from_param(node_logger):
    ws_root = os.getcwd()
    xacro_path = os.path.join(ws_root, "src/sfbot_can/description/urdf/my_robot.urdf.xacro")
    if not os.path.exists(xacro_path):
        xacro_path = os.path.join(ws_root, "install/sfbot_can/share/sfbot_can/description/urdf/my_robot.urdf.xacro")
    if not os.path.exists(xacro_path):
        node_logger.error(f"XACRO file not found: {xacro_path}")
        raise FileNotFoundError("XACRO file not found.")
    
    node_logger.info(f"Using XACRO file: {xacro_path}")
    temp_urdf = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w+')
    try:
        subprocess.run(["xacro", xacro_path], stdout=temp_urdf, check=True)
        temp_urdf.close()
        robot = rtb.Robot.URDF(temp_urdf.name)
    except Exception as e:
        node_logger.error(f"Failed to process XACRO or load robot model: {e}")
        raise
    finally:
        os.unlink(temp_urdf.name)
    
    node_logger.info(f"Robot model '{robot.name}' loaded with {robot.n} joints.")
    return robot

class RobustCartesianController(Node):
    def __init__(self, robot_model):
        super().__init__('robust_cartesian_controller')
        
        self.robot = robot_model
        # joint_trajectory_controller는 YAML에 정의된 1~6 순서를 따릅니다.
        self.joint_names = [f'link{i+1}_1_joint' for i in range(6)]
        self.current_joint_positions = np.zeros(self.robot.n)
        self.joint_state_received = False
        
        self.planned_path_waypoints = []
        self.actual_path_log = []
        self.is_recording = False
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.get_logger().info('🚀 Robust Cartesian Controller (JTC) initialized.')

    def joint_state_callback(self, msg):
        if not self.joint_state_received:
            try:
                # YAML에 정의된 순서대로 현재 관절 위치를 저장합니다.
                temp_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
                self.current_joint_positions = np.array([temp_positions[name] for name in self.joint_names])
                self.get_logger().info("First /joint_states message received. Current position is known.")
                self.joint_state_received = True
            except KeyError:
                # 가끔 순서가 다른 메시지가 올 수 있으므로 무시
                pass

        if self.is_recording:
            try:
                # 로깅 시에는 실제 /joint_states 순서를 따릅니다.
                log_joint_order = [
                    'link1_1_joint', 'link2_1_joint', 'link3_1_joint',
                    'link5_1_joint', 'link4_1_joint', 'link6_1_joint'
                ]
                temp_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
                log_positions = np.array([temp_positions[name] for name in log_joint_order])

                current_pose = self.robot.fkine(log_positions)
                self.actual_path_log.append(current_pose.t.tolist())
            except Exception:
                pass

    def solve_ik_robust(self, target_pose, q0=None):
        """
        다양한 방법을 시도하여 IK 해를 반드시 찾아내는 강력한 솔버
        """
        if q0 is None:
            q0 = self.current_joint_positions

        # 1단계: 현재 위치에서 LM 알고리즘 시도 (가장 빠름)
        sol, success, _, _, _ = self.robot.ik_LM(target_pose, q0=q0)
        if success:
            return sol, True
            
        # 2단계: 0점 위치에서 LM 알고리즘 시도
        sol, success, _, _, _ = self.robot.ik_LM(target_pose, q0=np.zeros(self.robot.n))
        if success:
            return sol, True

        # 3단계: 현재 위치에서 NR 알고리즘 시도 (다른 접근)
        try:
            sol, success, _, _, _ = self.robot.ik_NR(target_pose, q0=q0)
            if success:
                return sol, True
        except Exception:
            pass

        # 4단계: 무작위 위치에서 여러 번 시도 (최후의 수단)
        for _ in range(5):
            q_random = np.random.uniform(-np.pi, np.pi, self.robot.n)
            sol, success, _, _, _ = self.robot.ik_LM(target_pose, q0=q_random)
            if success:
                return sol, True

        self.get_logger().error(f"IK FAILED FOR ALL METHODS. Target: {target_pose.t}")
        return None, False

    def send_trajectory_goal(self, joint_positions_list, duration_list):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        for positions, duration in zip(joint_positions_list, duration_list):
            point = JointTrajectoryPoint()
            point.positions = positions.tolist()
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending a trajectory with {len(joint_positions_list)} points...')
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'Trajectory failed with error: {result.error_string}')
            return False

    def run_square_test(self):
        self.get_logger().info("="*10 + " ⬜ Robust Square Test (JTC) " + "="*10)
        
        side_length = 0.15
        center_x = 0.3
        center_y = 0.0
        z_height = 0.25

        waypoints_xyz = [
            [center_x - side_length / 2, center_y - side_length / 2, z_height],
            [center_x + side_length / 2, center_y - side_length / 2, z_height],
            [center_x + side_length / 2, center_y + side_length / 2, z_height],
            [center_x - side_length / 2, center_y + side_length / 2, z_height],
            [center_x - side_length / 2, center_y - side_length / 2, z_height]
        ]
        
        self.planned_path_waypoints = waypoints_xyz
        self.actual_path_log.clear()
        self.is_recording = True

        q_current = self.current_joint_positions
        trajectory_points = []
        trajectory_times = []
        total_time = 0.0
        duration_per_segment = 3.0

        for i, point_xyz in enumerate(waypoints_xyz):
            self.get_logger().info(f"--- Calculating IK for Waypoint {i+1}/{len(waypoints_xyz)} ---")
            target_pose = SE3(point_xyz[0], point_xyz[1], point_xyz[2])
            
            q_goal, success = self.solve_ik_robust(target_pose, q_current)
            
            if not success:
                self.get_logger().error(f"IK failed. Aborting test.")
                return

            trajectory_points.append(q_goal)
            total_time += duration_per_segment
            trajectory_times.append(total_time)
            q_current = q_goal
        
        if not self.send_trajectory_goal(trajectory_points, trajectory_times):
            self.get_logger().error("Test failed.")
        
        self.is_recording = False
        self.get_logger().info("="*10 + " ✅ Test Finished " + "="*10)
        # self.visualize_results() # 시각화는 필요 시 주석 해제

def main(args=None):
    rclpy.init(args=args)
    
    temp_node = rclpy.create_node('robot_loader_node')
    try:
        robot = load_robot_from_param(temp_node.get_logger())
    finally:
        temp_node.destroy_node()

    controller_node = RobustCartesianController(robot)

    try:
        wait_start_time = time.time()
        controller_node.get_logger().info("Waiting for robot's actual joint state...")
        while not controller_node.joint_state_received and time.time() - wait_start_time < 10.0:
            rclpy.spin_once(controller_node, timeout_sec=0.1)

        if not controller_node.joint_state_received:
             controller_node.get_logger().error("Could not receive /joint_states. Aborting.")
             return
            
        controller_node.run_square_test()
        
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()