#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import threading
import time
import sys
import os
import json
import numpy as np
import subprocess
import tempfile

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
    print("Required Python packages not installed. Please run:")
    print("pip install roboticstoolbox-python spatialmath-python matplotlib")
    sys.exit(1)

def load_robot_from_param(node_logger):
    """
    Loads the robot model from the user's 'sfbot_can' package.
    """
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

    # This order must match the actual order published by /joint_states
    # We force synchronization based on previous logs.
    actual_joint_names = [
        'link1_1_joint', 'link2_1_joint', 'link3_1_joint',
        'link4_1_joint', 'link5_1_joint', 'link6_1_joint'
    ]
    node_logger.info(f"Robot model '{robot.name}' loaded with {robot.n} joints.")
    node_logger.info(f"Using forced-synchronized joint names: {actual_joint_names}")
    return robot, actual_joint_names

class AdvancedCartesianController(Node):
    """
    Final, robust, action-based Cartesian controller with self-testing.
    """
    def __init__(self, robot_model, joint_names_list):
        super().__init__('advanced_cartesian_controller')
        
        self.robot = robot_model
        self.joint_names = joint_names_list
        self.current_joint_positions = np.zeros(self.robot.n)
        self.joint_state_received = False
        
        self.planned_path_waypoints = []
        self.actual_path_log = []
        self.is_recording = False
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.get_logger().info('🚀 Action-based Cartesian Controller initialized.')

    def joint_state_callback(self, msg):
        if not hasattr(self, '_joint_mapping'):
            try:
                self._joint_mapping = [msg.name.index(j) for j in self.joint_names]
            except ValueError:
                return # Wait for a message with the correct names

        self.current_joint_positions = np.array([msg.position[i] for i in self._joint_mapping])
        
        if not self.joint_state_received:
            self.get_logger().info("First /joint_states message received. Robot position is known.")
            self.joint_state_received = True

        if self.is_recording:
            try:
                current_pose = self.robot.fkine(self.current_joint_positions)
                self.actual_path_log.append(current_pose.t.tolist())
            except Exception:
                pass

    def solve_ik(self, target_pose, current_q=None):
        initial_guesses = [current_q] if current_q is not None and np.any(current_q) else []
        initial_guesses.append(np.zeros(self.robot.n))
        for _ in range(5): 
            initial_guesses.append(np.random.uniform(-np.pi, np.pi, self.robot.n))
        
        for q0 in initial_guesses:
            sol, success, _, _, _ = self.robot.ik_LM(target_pose, q0=q0, joint_limits=True)
            if success:
                return sol, True
        self.get_logger().error(f"IK Failed. Target: {target_pose.t}")
        return None, False

    def send_trajectory_goal(self, joint_positions, duration):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending goal (move in {duration}s)...')
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Manually spin until the future is done
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return False

        self.get_logger().info('Goal accepted by server.')
        get_result_future = goal_handle.get_result_async()
        
        while rclpy.ok() and not get_result_future.done():
            time.sleep(0.1)

        result = get_result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Goal reached successfully!')
            return True
        else:
            self.get_logger().error(f'Goal failed with error: {result.error_string}')
            return False

    def run_square_test(self):
        self.get_logger().info("="*10 + " 🔲 Action-based Square Test Start " + "="*10)
        
        side_length = 0.15
        center_x = 0.3
        center_y = 0.0
        z_height = 0.25

        waypoints = [
            [center_x - side_length / 2, center_y - side_length / 2, z_height],
            [center_x + side_length / 2, center_y - side_length / 2, z_height],
            [center_x + side_length / 2, center_y + side_length / 2, z_height],
            [center_x - side_length / 2, center_y + side_length / 2, z_height],
            [center_x - side_length / 2, center_y - side_length / 2, z_height]
        ]
        
        self.planned_path_waypoints = waypoints
        self.actual_path_log.clear()
        self.is_recording = True

        q_current = self.current_joint_positions
        
        for i, point in enumerate(waypoints):
            self.get_logger().info(f"--- Processing Waypoint {i+1}/{len(waypoints)}: {point} ---")
            target_pose = SE3(point[0], point[1], point[2])
            
            q_goal, success = self.solve_ik(target_pose, q_current)
            
            if not success:
                self.get_logger().error(f"IK failed for waypoint {i+1}. Aborting test.")
                break

            duration = 3.0
            if not self.send_trajectory_goal(q_goal, duration):
                self.get_logger().error("Trajectory execution failed. Aborting test.")
                break

            q_current = q_goal
            time.sleep(0.5)

        self.is_recording = False
        self.get_logger().info("="*10 + " ✅ Test Finished " + "="*10)
        self.visualize_results()

    def visualize_results(self):
        self.get_logger().info("📈 Visualizing results...")
        
        planned = np.array(self.planned_path_waypoints)
        actual = np.array(self.actual_path_log)

        if actual.size == 0:
            self.get_logger().error("No actual path data was logged. Cannot visualize.")
            return

        def plot_process():
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(planned[:, 0], planned[:, 1], planned[:, 2], 'g--', marker='o', label='Planned Path')
            ax.plot(actual[:, 0], actual[:, 1], actual[:, 2], 'b-', label='Actual Path')
            ax.scatter(planned[0, 0], planned[0, 1], planned[0, 2], c='red', s=100, label='Start Point', marker='*')
            
            ax.set_title('Planned vs. Actual Robot Path')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.legend()
            ax.grid(True)
            ax.set_aspect('equal')
            
            plt.show()

        plot_thread = threading.Thread(target=plot_process)
        plot_thread.start()
        
def main(args=None):
    rclpy.init(args=args)
    
    temp_node = rclpy.create_node('robot_loader_node')
    try:
        robot, joint_names = load_robot_from_param(temp_node.get_logger())
    except Exception as e:
        temp_node.get_logger().fatal(f"Fatal error during robot loading: {e}")
        return
    finally:
        temp_node.destroy_node()

    controller_node = AdvancedCartesianController(robot, joint_names)
    
    if '--test-square' in sys.argv:
        try:
            wait_start_time = time.time()
            controller_node.get_logger().info("Waiting for robot's actual joint state... (max 10s)")
            while not controller_node.joint_state_received and time.time() - wait_start_time < 10.0:
                rclpy.spin_once(controller_node, timeout_sec=0.1)

            if not controller_node.joint_state_received:
                 controller_node.get_logger().error("Could not receive /joint_states. Is the robot driver running? Aborting test.")
                 return
            
            controller_node.run_square_test()
            
            controller_node.get_logger().info("Close the plot window to exit the program.")
            while rclpy.ok():
                time.sleep(1)
            
        except KeyboardInterrupt:
            pass
        finally:
            controller_node.destroy_node()
            rclpy.shutdown()
    else:
        # Normal controller mode
        rclpy.spin(controller_node)
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()