#!/usr/bin/env python3
"""
TF 기반 실제 경로 시각화
- 실제 robot_state_publisher에서 나오는 TF 사용
- Forward Kinematics 계산 없이 정확한 엔드이펙터 위치
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import datetime
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class TFBasedVisualizer(Node):
    def __init__(self):
        super().__init__('tf_based_visualizer')
        
        # TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 명령 구독자
        self.command_sub = self.create_subscription(
            String, '/data_collector_command', self.command_callback, 10)
        
        # 데이터 저장
        self.end_effector_positions = []
        self.timestamps = []
        self.start_time = None
        self.collecting = False
        
        # TF 프레임 이름들 (자동 감지 시도)
        self.base_frame = 'base_link'
        self.end_effector_frame = None
        self.possible_ee_frames = ['tool0', 'link6_1', 'end_effector', 'tcp', 'tool_frame']
        
        # 계획 경로
        self.planned_square = np.array([
            [0.25, -0.01, 0.4],  # 좌하
            [0.27, -0.01, 0.4],  # 우하
            [0.27, 0.01, 0.4],   # 우상
            [0.25, 0.01, 0.4],   # 좌상
            [0.25, -0.01, 0.4]   # 시작점
        ])
        
        self.get_logger().info('📡 TF-based Path Visualizer Started')
        self.get_logger().info('🔍 Detecting end-effector frame...')
        
        # 엔드이펙터 프레임 자동 감지
        self.create_timer(1.0, self.detect_end_effector_frame)
        
        # 데이터 수집 타이머 (비활성화 상태로 시작)
        self.data_timer = None
        
        self.get_logger().info('🎮 Commands:')
        self.get_logger().info('  Start: ros2 topic pub /data_collector_command std_msgs/String "data: start"')
        self.get_logger().info('  Stop: ros2 topic pub /data_collector_command std_msgs/String "data: stop"')
    
    def detect_end_effector_frame(self):
        """엔드이펙터 프레임 자동 감지"""
        if self.end_effector_frame is not None:
            return  # 이미 찾았으면 리턴
        
        for frame in self.possible_ee_frames:
            try:
                # 현재 시간에서 transform 시도
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                
                self.end_effector_frame = frame
                self.get_logger().info(f'✅ End-effector frame detected: {frame}')
                
                # 현재 위치 출력
                pos = transform.transform.translation
                self.get_logger().info(f'📍 Current position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
                
                return
                
            except Exception as e:
                continue
        
        # 5초 후에도 못 찾으면 수동 설정 요청
        if not hasattr(self, '_manual_request_sent'):
            self.get_logger().warn('⚠️  Could not auto-detect end-effector frame')
            self.get_logger().warn('Available frames:')
            self.get_logger().warn('ros2 topic echo /tf_static --once')
            self.get_logger().warn('Set manually: self.end_effector_frame = "your_frame_name"')
            self._manual_request_sent = True
    
    def command_callback(self, msg):
        """명령 처리"""
        command = msg.data.lower().strip()
        
        if command == 'start':
            if not self.collecting and self.end_effector_frame:
                self.start_collecting()
            else:
                if not self.end_effector_frame:
                    self.get_logger().error('End-effector frame not detected yet!')
                else:
                    self.get_logger().warn('Already collecting!')
                
        elif command == 'stop':
            if self.collecting:
                self.stop_collecting()
            else:
                self.get_logger().warn('Not collecting!')
    
    def start_collecting(self):
        """데이터 수집 시작"""
        self.collecting = True
        self.start_time = time.time()
        self.end_effector_positions = []
        self.timestamps = []
        
        # 10Hz로 TF 데이터 수집
        self.data_timer = self.create_timer(0.1, self.collect_tf_data)
        
        self.get_logger().info('🎬 TF data collection started!')
        self.get_logger().info(f'📡 Tracking: {self.base_frame} → {self.end_effector_frame}')
    
    def stop_collecting(self):
        """데이터 수집 종료"""
        self.collecting = False
        
        if self.data_timer:
            self.data_timer.cancel()
            self.data_timer = None
        
        self.get_logger().info(f'🛑 Collection stopped - {len(self.end_effector_positions)} points')
        
        if len(self.end_effector_positions) > 10:
            self.create_visualization()
        else:
            self.get_logger().error('Insufficient data')
    
    def collect_tf_data(self):
        """TF 데이터 수집"""
        if not self.collecting or not self.end_effector_frame:
            return
        
        try:
            # 현재 시간의 transform 가져오기
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_frame, 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            # 위치 정보 추출
            pos = transform.transform.translation
            position = [pos.x, pos.y, pos.z]
            
            self.end_effector_positions.append(position)
            self.timestamps.append(time.time() - self.start_time)
            
            # 진행 상황 출력
            if len(self.end_effector_positions) % 50 == 0:
                self.get_logger().info(f'📈 {len(self.end_effector_positions)} TF points collected')
                self.get_logger().info(f'📍 Current: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
        
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
    
    def create_visualization(self):
        """시각화 생성"""
        self.get_logger().info('📊 Creating TF-based visualization...')
        
        actual_path = np.array(self.end_effector_positions)
        timestamps = np.array(self.timestamps)
        
        # 3개 서브플롯
        fig = plt.figure(figsize=(18, 6))
        
        # Plot 1: Planned vs Actual Path (2D XY view)
        ax1 = fig.add_subplot(131)
        
        # 계획 경로
        ax1.plot(self.planned_square[:, 0], self.planned_square[:, 1], 'b-o', 
                linewidth=3, markersize=8, label='Planned Path', alpha=0.8)
        
        # 실제 경로 (TF 기반)
        ax1.plot(actual_path[:, 0], actual_path[:, 1], 'r-', 
                linewidth=2, label='Actual Path (TF)', alpha=0.7)
        
        # 시작점과 끝점
        ax1.plot(self.planned_square[0, 0], self.planned_square[0, 1], 'go', 
                markersize=12, label='Start Point')
        ax1.plot(actual_path[0, 0], actual_path[0, 1], 'ro', 
                markersize=10, label='Actual Start')
        
        # 모서리 번호
        for i, corner in enumerate(self.planned_square[:-1]):
            x, y = corner[0], corner[1]
            ax1.annotate(f'{i+1}', (x, y), xytext=(5, 5), textcoords='offset points',
                        fontsize=10, color='blue', weight='bold')
        
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('Planned vs Actual Path (TF-based)', fontsize=14, weight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: 3D 궤적
        ax2 = fig.add_subplot(132, projection='3d')
        
        # 계획 경로 3D
        ax2.plot(self.planned_square[:, 0], self.planned_square[:, 1], self.planned_square[:, 2],
                'b-o', linewidth=3, markersize=8, label='Planned Path')
        
        # 실제 경로 3D
        ax2.plot(actual_path[:, 0], actual_path[:, 1], actual_path[:, 2],
                'r-', linewidth=2, label='Actual Path (TF)')
        
        ax2.set_xlabel('X (m)', fontsize=12)
        ax2.set_ylabel('Y (m)', fontsize=12)
        ax2.set_zlabel('Z (m)', fontsize=12)
        ax2.set_title('3D Trajectory', fontsize=14, weight='bold')
        ax2.legend(fontsize=10)
        
        # Plot 3: 정확도 분석
        ax3 = fig.add_subplot(133)
        
        # 각 계획 모서리별 오차 계산
        corner_errors = []
        for planned_corner in self.planned_square[:-1]:
            distances = np.linalg.norm(actual_path - planned_corner, axis=1)
            min_error = np.min(distances) * 1000  # mm 단위
            corner_errors.append(min_error)
        
        # 막대 그래프
        bars = ax3.bar(range(1, len(corner_errors) + 1), corner_errors, 
                      color=['skyblue', 'lightgreen', 'orange', 'pink'], alpha=0.7)
        
        # 값 표시
        for bar, error in zip(bars, corner_errors):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                    f'{error:.1f}', ha='center', va='bottom', fontsize=10)
        
        ax3.set_xlabel('Corner Number', fontsize=12)
        ax3.set_ylabel('Position Error (mm)', fontsize=12)
        ax3.set_title('Corner Accuracy (TF-based)', fontsize=14, weight='bold')
        ax3.grid(True, alpha=0.3)
        
        # 통계 텍스트
        avg_error = np.mean(corner_errors)
        max_error = np.max(corner_errors)
        stats_text = f'Avg Error: {avg_error:.1f}mm\nMax Error: {max_error:.1f}mm'
        ax3.text(0.7, 0.9, stats_text, transform=ax3.transAxes, 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                fontsize=10, verticalalignment='top')
        
        # 파일 저장
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'tf_based_experiment_{timestamp}'
        
        plt.tight_layout()
        plt.savefig(f'{filename}.png', dpi=300, bbox_inches='tight')
        
        # 통계 출력
        total_time = timestamps[-1]
        avg_freq = len(self.end_effector_positions) / total_time
        
        print(f"\n📊 TF-based Experiment Results:")
        print(f"   Total Time: {total_time:.1f} seconds")
        print(f"   Data Points: {len(self.end_effector_positions)} points")
        print(f"   Average Frequency: {avg_freq:.1f} Hz")
        print(f"   TF Frames: {self.base_frame} → {self.end_effector_frame}")
        print(f"   Average Path Error: {avg_error:.1f} mm")
        print(f"   Maximum Path Error: {max_error:.1f} mm")
        print(f"   Files Saved: {filename}.png, {filename}_data.json")
        
        # 데이터 저장
        data = {
            'timestamps': timestamps.tolist(),
            'actual_path': actual_path.tolist(),
            'planned_path': self.planned_square.tolist(),
            'tf_frames': {
                'base_frame': self.base_frame,
                'end_effector_frame': self.end_effector_frame
            },
            'statistics': {
                'total_time': total_time,
                'data_points': len(self.end_effector_positions),
                'avg_frequency': avg_freq,
                'corner_errors': corner_errors,
                'avg_error': avg_error,
                'max_error': max_error
            }
        }
        
        import json
        with open(f'{filename}_data.json', 'w') as f:
            json.dump(data, f, indent=2)
        
        plt.show()
        self.get_logger().info(f'✅ TF-based visualization complete! {filename}.png')


def main():
    rclpy.init()
    
    try:
        visualizer = TFBasedVisualizer()
        
        print("\n📡 TF-based Path Visualizer Ready!")
        print("\n📋 Usage:")
        print("1. Wait for end-effector frame detection")
        print("2. Start: ros2 topic pub /data_collector_command std_msgs/String \"data: start\"")
        print("3. Execute robot movements")
        print("4. Stop: ros2 topic pub /data_collector_command std_msgs/String \"data: stop\"")
        print("5. View accurate TF-based results!")
        print("\n🔍 Frame detection:")
        print("   Trying: tool0, link6_1, end_effector, tcp, tool_frame")
        print("   Check: ros2 run tf2_tools view_frames")
        
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("Terminated")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()