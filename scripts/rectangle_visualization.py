#!/usr/bin/env python3
"""
수동 데이터 수집기
- 내가 언제 시작할지 결정
- 내가 언제 끝낼지 결정
- 터미널에서 명령어로 제어
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import datetime

class ManualDataCollector(Node):
    def __init__(self):
        super().__init__('manual_data_collector')
        
        # 구독자
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 발행자 (시작/종료 명령용)
        self.command_sub = self.create_subscription(
            String, '/data_collector_command', self.command_callback, 10)
        
        # 데이터 저장
        self.joint_positions = []
        self.timestamps = []
        self.start_time = None
        self.collecting = False
        
        # 목표 사각형 경로
        self.target_square = [
            [0.25, -0.01, 0.4],  # 좌하
            [0.27, -0.01, 0.4],  # 우하
            [0.27, 0.01, 0.4],   # 우상
            [0.25, 0.01, 0.4],   # 좌상
            [0.25, -0.01, 0.4]   # 시작점
        ]
        
        self.get_logger().info('📊 수동 데이터 수집기 시작')
        self.get_logger().info('')
        self.get_logger().info('🎮 사용법:')
        self.get_logger().info('  시작: ros2 topic pub /data_collector_command std_msgs/String "data: start"')
        self.get_logger().info('  종료: ros2 topic pub /data_collector_command std_msgs/String "data: stop"')
        self.get_logger().info('')
        self.get_logger().info('📋 실험 순서:')
        self.get_logger().info('1. 안전한 위치로 이동')
        self.get_logger().info('2. "start" 명령')
        self.get_logger().info('3. 사각형 그리기 명령')
        self.get_logger().info('4. "stop" 명령')
        self.get_logger().info('')
    
    def command_callback(self, msg):
        """명령 처리"""
        command = msg.data.lower().strip()
        
        if command == 'start':
            if not self.collecting:
                self.start_collecting()
            else:
                self.get_logger().warn('이미 수집 중입니다!')
                
        elif command == 'stop':
            if self.collecting:
                self.stop_collecting()
            else:
                self.get_logger().warn('수집이 시작되지 않았습니다!')
                
        else:
            self.get_logger().warn(f'알 수 없는 명령: {command}')
    
    def start_collecting(self):
        """데이터 수집 시작"""
        self.collecting = True
        self.start_time = time.time()
        self.joint_positions = []
        self.timestamps = []
        
        self.get_logger().info('🎬 데이터 수집 시작!')
        self.get_logger().info('이제 사각형 그리기 명령을 주세요')
    
    def stop_collecting(self):
        """데이터 수집 종료"""
        self.collecting = False
        
        self.get_logger().info(f'🛑 데이터 수집 종료 - 총 {len(self.joint_positions)}개 포인트')
        
        if len(self.joint_positions) > 10:
            self.create_visualization()
        else:
            self.get_logger().error('데이터가 부족합니다')
    
    def joint_state_callback(self, msg):
        """관절 상태 수집"""
        if not self.collecting:
            return
            
        try:
            # 처음 6개 관절
            joint_pos = list(msg.position[:6])
            if len(joint_pos) < 6:
                joint_pos.extend([0.0] * (6 - len(joint_pos)))
            
            self.joint_positions.append(joint_pos)
            self.timestamps.append(time.time() - self.start_time)
            
            # 진행 상황 (조용히)
            if len(self.joint_positions) % 200 == 0:
                self.get_logger().info(f'📈 {len(self.joint_positions)}개 수집됨...')
                
        except Exception as e:
            self.get_logger().warn(f'수집 오류: {e}')
    
    def simple_forward_kinematics(self, joint_positions):
        """Simple Forward Kinematics estimation"""
        # Estimated link lengths (adjust for your robot)
        L1, L2, L3, L4, L5, L6 = 0.1, 0.2, 0.2, 0.1, 0.1, 0.05
        
        q = joint_positions
        
        # Simple calculation (needs adjustment based on actual robot DH parameters)
        x = L2 * np.cos(q[0]) * np.cos(q[1]) + L3 * np.cos(q[0]) * np.cos(q[1] + q[2])
        y = L2 * np.sin(q[0]) * np.cos(q[1]) + L3 * np.sin(q[0]) * np.cos(q[1] + q[2])
        z = L1 + L2 * np.sin(q[1]) + L3 * np.sin(q[1] + q[2])
        
        return [x, y, z]

    def create_visualization(self):
        """Create visualization with planned vs actual path comparison"""
        self.get_logger().info('📊 Creating visualization...')
        
        joint_data = np.array(self.joint_positions)
        timestamps = np.array(self.timestamps)
        
        # Calculate actual end-effector positions using FK
        actual_positions = []
        for joint_pos in self.joint_positions:
            ee_pos = self.simple_forward_kinematics(joint_pos)
            actual_positions.append(ee_pos)
        actual_path = np.array(actual_positions)
        
        # 3 subplots
        fig = plt.figure(figsize=(18, 6))
        
        # Plot 1: Planned vs Actual Path (2D XY view)
        ax1 = fig.add_subplot(131)
        
        # Planned square path
        planned_square = np.array(self.target_square)
        ax1.plot(planned_square[:, 0], planned_square[:, 1], 'b-o', linewidth=3, markersize=8, 
                label='Planned Path', alpha=0.8)
        
        # Actual path
        if len(actual_path) > 0:
            ax1.plot(actual_path[:, 0], actual_path[:, 1], 'r-', linewidth=2, 
                    label='Actual Path', alpha=0.7)
        
        # Mark start and end points
        ax1.plot(planned_square[0, 0], planned_square[0, 1], 'go', markersize=12, label='Start Point')
        ax1.plot(planned_square[-1, 0], planned_square[-1, 1], 'ro', markersize=12, label='End Point')
        
        # Corner numbers
        for i, corner in enumerate(planned_square[:-1]):
            x, y = corner[0], corner[1]
            ax1.annotate(f'{i+1}', (x, y), xytext=(5, 5), textcoords='offset points',
                        fontsize=10, color='blue', weight='bold')
        
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('Planned vs Actual Path (XY View)', fontsize=14, weight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: Joint Trajectories
        ax2 = fig.add_subplot(132)
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
        
        for i in range(6):
            ax2.plot(timestamps, joint_data[:, i], color=colors[i], 
                    label=f'Joint {i+1}', linewidth=2)
        
        ax2.set_xlabel('Time (seconds)', fontsize=12)
        ax2.set_ylabel('Joint Angle (rad)', fontsize=12)
        ax2.set_title('Joint Trajectories', fontsize=14, weight='bold')
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Path Accuracy Analysis
        ax3 = fig.add_subplot(133)
        
        # Calculate position errors for each planned corner
        corner_errors = []
        if len(actual_path) > 0:
            for i, planned_corner in enumerate(planned_square[:-1]):  # Exclude last point (return to start)
                # Find closest actual position to this planned corner
                distances = np.linalg.norm(actual_path - planned_corner, axis=1)
                min_error = np.min(distances) * 1000  # Convert to mm
                corner_errors.append(min_error)
            
            # Bar chart of corner errors
            bars = ax3.bar(range(1, len(corner_errors) + 1), corner_errors, 
                          color=['skyblue', 'lightgreen', 'orange', 'pink'], alpha=0.7)
            
            # Add error values on bars
            for bar, error in zip(bars, corner_errors):
                ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                        f'{error:.1f}', ha='center', va='bottom', fontsize=10)
            
            ax3.set_xlabel('Corner Number', fontsize=12)
            ax3.set_ylabel('Position Error (mm)', fontsize=12)
            ax3.set_title('Corner Accuracy Analysis', fontsize=14, weight='bold')
            ax3.grid(True, alpha=0.3)
            
            # Add statistics text
            avg_error = np.mean(corner_errors)
            max_error = np.max(corner_errors)
            stats_text = f'Avg Error: {avg_error:.1f}mm\nMax Error: {max_error:.1f}mm'
            ax3.text(0.7, 0.9, stats_text, transform=ax3.transAxes, 
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    fontsize=10, verticalalignment='top')
        else:
            ax3.text(0.5, 0.5, 'No actual path data', ha='center', va='center', 
                    transform=ax3.transAxes, fontsize=16)
        
        # 파일 저장
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'manual_experiment_{timestamp}'
        
        plt.tight_layout()
        plt.savefig(f'{filename}.png', dpi=300, bbox_inches='tight')
        
        # 통계 출력
        total_time = timestamps[-1]
        avg_freq = len(self.joint_positions) / total_time
        
        # Joint movement analysis
        joint_ranges = []
        for i in range(6):
            joint_range = np.max(joint_data[:, i]) - np.min(joint_data[:, i])
            joint_ranges.append(joint_range)
        
        max_range_joint = np.argmax(joint_ranges) + 1
        max_range_value = np.max(joint_ranges)
        
        print(f"\n📊 Experiment Results:")
        print(f"   Total Time: {total_time:.1f} seconds")
        print(f"   Data Points: {len(self.joint_positions)} points")
        print(f"   Average Frequency: {avg_freq:.1f} Hz")
        print(f"   Most Active Joint: Joint {max_range_joint} ({max_range_value:.3f} rad)")
        
        if len(actual_path) > 0:
            # Path accuracy statistics
            corner_errors = []
            for planned_corner in planned_square[:-1]:
                distances = np.linalg.norm(actual_path - planned_corner, axis=1)
                min_error = np.min(distances) * 1000  # mm
                corner_errors.append(min_error)
            
            avg_path_error = np.mean(corner_errors)
            max_path_error = np.max(corner_errors)
            print(f"   Average Path Error: {avg_path_error:.1f} mm")
            print(f"   Maximum Path Error: {max_path_error:.1f} mm")
        
        print(f"   Files Saved: {filename}.png, {filename}_data.json")
        
        # 데이터도 저장
        data = {
            'timestamps': timestamps.tolist(),
            'joint_positions': joint_data.tolist(),
            'actual_path': actual_path.tolist() if len(actual_path) > 0 else [],
            'planned_path': planned_square.tolist(),
            'statistics': {
                'total_time': total_time,
                'data_points': len(self.joint_positions),
                'avg_frequency': avg_freq,
                'joint_ranges': joint_ranges,
                'path_errors': corner_errors if len(actual_path) > 0 else []
            }
        }
        
        import json
        with open(f'{filename}_data.json', 'w') as f:
            json.dump(data, f, indent=2)
        
        plt.show()
        self.get_logger().info(f'✅ Visualization complete! {filename}.png')


def main():
    rclpy.init()
    
    try:
        collector = ManualDataCollector()
        
        print("\n🎯 수동 데이터 수집기 준비 완료!")
        print("\n📋 실험 순서:")
        print("1. 안전한 위치로 이동:")
        print("   ros2 topic pub --once /cartesian_command geometry_msgs/PointStamped '{")
        print("     header: {frame_id: \"base_link\"},")
        print("     point: {x: 0.25, y: 0.0, z: 0.4}")
        print("   }'")
        print("")
        print("2. 데이터 수집 시작:")
        print('   ros2 topic pub --once /data_collector_command std_msgs/String "data: start"')
        print("")
        print("3. 사각형 그리기:")
        print("   ros2 topic pub --once /cartesian_path_command std_msgs/String '{")
        print('     data: "{\\"type\\": \\"waypoints\\", \\"waypoints\\": [[0.25, -0.01, 0.4], [0.27, -0.01, 0.4], [0.27, 0.01, 0.4], [0.25, 0.01, 0.4], [0.25, -0.01, 0.4]], \\"duration\\": 15.0}"')
        print("   }'")
        print("")
        print("4. 데이터 수집 종료:")
        print('   ros2 topic pub --once /data_collector_command std_msgs/String "data: stop"')
        print("")
        print("🎮 제어 명령:")
        print("   시작: ros2 topic pub /data_collector_command std_msgs/String \"data: start\"")
        print("   종료: ros2 topic pub /data_collector_command std_msgs/String \"data: stop\"")
        
        rclpy.spin(collector)
        
    except KeyboardInterrupt:
        print("종료됨")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()