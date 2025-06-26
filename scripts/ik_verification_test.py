#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import sys
import numpy as np
import subprocess
import tempfile

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
except ImportError:
    print("Required Python packages not installed. Please run: pip install roboticstoolbox-python spatialmath-python")
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

class IKVerificationNode(Node):
    def __init__(self, robot_model):
        super().__init__('ik_verification_node')
        self.robot = robot_model
        # forward_position_controller는 YAML에 정의된 순서(1~6)를 따릅니다.
        self.joint_names = [f'link{i+1}_1_joint' for i in range(6)]
        self.current_joint_positions = None
        self.joint_state_received = False

        self.command_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info('🚀 IK Verification Node initialized.')

    def joint_state_callback(self, msg):
        if not self.joint_state_received:
            # YAML에 정의된 순서대로 현재 관절 위치를 저장합니다.
            temp_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
            self.current_joint_positions = np.array([temp_positions[name] for name in self.joint_names])
            self.get_logger().info("First /joint_states message received. Current position is known.")
            self.joint_state_received = True

    def solve_and_send(self, target_xyz):
        self.get_logger().info(f"Target XYZ position: {target_xyz}")
        
        target_pose = SE3(target_xyz[0], target_xyz[1], target_xyz[2])
        
        q_goal, success, _, _, _ = self.robot.ik_LM(target_pose, q0=self.current_joint_positions)
        
        if not success:
            self.get_logger().error("!!! IK solution NOT FOUND. The target is likely unreachable.")
            return

        self.get_logger().info("==============================================")
        self.get_logger().info(f"IK Solution Found (in Radians):")
        self.get_logger().info(f"  Joint 1: {q_goal[0]:.4f}")
        self.get_logger().info(f"  Joint 2: {q_goal[1]:.4f}")
        self.get_logger().info(f"  Joint 3: {q_goal[2]:.4f}")
        self.get_logger().info(f"  Joint 4: {q_goal[3]:.4f}")
        self.get_logger().info(f"  Joint 5: {q_goal[4]:.4f}")
        self.get_logger().info(f"  Joint 6: {q_goal[5]:.4f}")
        self.get_logger().info("==============================================")
        self.get_logger().info("Sending this command to /forward_position_controller/commands...")

        cmd_msg = Float64MultiArray()
        cmd_msg.data = q_goal.tolist()
        self.command_pub.publish(cmd_msg)
        self.get_logger().info("Command published.")

def main(args=None):
    rclpy.init(args=args)
    
    temp_node = rclpy.create_node('robot_loader_node')
    try:
        robot = load_robot_from_param(temp_node.get_logger())
    except Exception as e:
        temp_node.get_logger().fatal(f"Fatal error during robot loading: {e}")
        return
    finally:
        temp_node.destroy_node()

    verifier_node = IKVerificationNode(robot)

    try:
        wait_start_time = time.time()
        verifier_node.get_logger().info("Waiting for robot's actual joint state... (max 10s)")
        while not verifier_node.joint_state_received and time.time() - wait_start_time < 10.0:
            rclpy.spin_once(verifier_node, timeout_sec=0.1)

        if not verifier_node.joint_state_received:
             verifier_node.get_logger().error("Could not receive /joint_states. Is the robot driver running? Aborting test.")
             return
        
        # 매우 간단하고 도달 가능해 보이는 목표 지점 설정
        simple_target = [0.25, 0.1, 0.3]
        verifier_node.solve_and_send(simple_target)
        
        # 명령이 전달되고 로봇이 움직일 시간을 줌
        time.sleep(5)

    except KeyboardInterrupt:
        pass
    finally:
        verifier_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()