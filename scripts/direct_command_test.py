#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class DirectCommandTester(Node):
    def __init__(self):
        super().__init__('direct_command_tester')
        
        # forward_position_controller가 사용하는 토픽에 직접 발행
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # YAML 파일에 정의된 순서와 동일해야 함
        # 이 컨트롤러는 YAML 순서를 잘 따릅니다.
        self.joint_names = [
            'link1_1_joint', 'link2_1_joint', 'link3_1_joint',
            'link4_1_joint', 'link5_1_joint', 'link6_1_joint'
        ]
        self.get_logger().info('Direct Command Tester has been started.')
        self.get_logger().info('Will publish a command in 2 seconds...')

    def send_command(self):
        # 목표: 2번 관절만 0.5 라디안 (약 28도)으로 이동
        # forward_position_controller는 YAML에 정의된 1~6 순서를 따릅니다.
        command_positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = command_positions
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command published: move joint 2 to 0.5 rad')

def main(args=None):
    rclpy.init(args=args)
    command_tester = DirectCommandTester()
    
    # 노드가 뜨고 퍼블리셔가 준비될 시간을 줌
    time.sleep(2.0)
    
    # 명령 발행
    command_tester.send_command()
    
    # 메시지가 확실히 전송되도록 잠시 더 대기
    time.sleep(1.0)
    
    command_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()