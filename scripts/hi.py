#!/usr/bin/env python3
"""
정밀도 향상된 TF 기반 실제 경로 시각화
- 웨이포인트별 도달 확인 및 검증
- 실시간 정확도 모니터링
- 자동 재시도 시스템
- 상세한 성능 분석
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
import matplotlib.pyplot as plt
import time
import subprocess
import threading
from datetime import datetime
import json


"""
    1. # 1. 정밀 시각화 실행
/home/yj/ros2_venv/bin/python /home/yj/ros2_ws/src/sfbot_can/scripts/precision_tf_visualizer.py
    2. 안전한 위치 이동
    ros2 topic pub --once /cartesian_command geometry_msgs/PointStamped '{header: {frame_id: "base_link"}, point: {x: 0.25, y: 0.0, z: 0.4}}'
    # 3. 또는 수동 데이터 수집
    ros2 topic pub --once /precision_command std_msgs/String "data: collect"
    # ... 로봇 움직임 명령들 실행 ...

    ros2 topic pub --once /cartesian_path_command std_msgs/String '{data: "{\"type\": \"waypoints\", \"waypoints\": [[0.25, -0.01, 0.4], [0.27, -0.01, 0.4], [0.27, 0.01, 0.4], [0.25, 0.01, 0.4], [0.25, -0.01, 0.4]], \"duration\": 15.0}"}'

    ros2 topic pub --once /precision_command std_msgs/String "data: stop"
"""
class PrecisionTFVisualizer(Node):
    def __init__(self):
        super().__init__('precision_tf_visualizer')
        
        # TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 명령 구독자 & 발행자
        self.command_sub = self.create_subscription(
            String, '/precision_command', self.precision_command_callback, 10)
        
        self.status_pub = self.create_publisher(String, '/precision_status', 10)
        
        # 데이터 저장
        self.all_positions = []  # 전체 실험 동안의 위치
        self.waypoint_data = []  # 각 웨이포인트별 상세 데이터
        self.timestamps = []
        self.start_time = None
        self.collecting = False
        
        # TF 프레임 설정
        self.base_frame = 'base_link'
        self.end_effector_frame = None
        self.possible_ee_frames = ['tool0', 'link6_1', 'end_effector', 'tcp', 'tool_frame']
        
        # 정밀 사각형 경로 (더 세밀한 간격)
        self.precision_square = [
            [0.25, -0.01, 0.4],   # Corner 1: 좌하
            [0.27, -0.01, 0.4],   # Corner 2: 우하  
            [0.27, 0.01, 0.4],    # Corner 3: 우상
            [0.25, 0.01, 0.4],    # Corner 4: 좌상
            [0.25, -0.01, 0.4]    # Corner 5: 시작점 복귀
        ]
        
        # 정밀도 설정
        self.position_tolerance = 0.001  # 1mm 허용 오차 (더 엄격하게)
        self.settle_tolerance = 0.0005   # 0.5mm 안정화 허용 오차
        self.max_wait_time = 20.0        # 20초 최대 대기
        self.settle_time = 2.0           # 2초 안정화 시간
        self.monitoring_freq = 50        # 50Hz 모니터링 (더 정밀하게)
        self.retry_attempts = 2          # 실패 시 재시도 횟수
        
        # 실험 상태
        self.current_waypoint_index = 0
        self.experiment_active = False
        self.experiment_log = {
            'start_time': None,
            'waypoints': [],
            'overall_stats': {}
        }
        
        self.get_logger().info('🎯 Precision TF-based Visualizer Started')
        self.get_logger().info('⚙️  Settings:')
        self.get_logger().info(f'   Position Tolerance: {self.position_tolerance*1000:.1f}mm')
        self.get_logger().info(f'   Settle Tolerance: {self.settle_tolerance*1000:.1f}mm')
        self.get_logger().info(f'   Monitoring Frequency: {self.monitoring_freq}Hz')
        
        # 엔드이펙터 프레임 자동 감지
        self.create_timer(1.0, self.detect_end_effector_frame)
        
        self.get_logger().info('🎮 Commands:')
        self.get_logger().info('  Start Precision Test: ros2 topic pub /precision_command std_msgs/String "data: start"')
        self.get_logger().info('  Manual Collection: ros2 topic pub /precision_command std_msgs/String "data: collect"')
        self.get_logger().info('  Stop Collection: ros2 topic pub /precision_command std_msgs/String "data: stop"')
    
    def detect_end_effector_frame(self):
        """엔드이펙터 프레임 자동 감지"""
        if self.end_effector_frame is not None:
            return
        
        for frame in self.possible_ee_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                
                self.end_effector_frame = frame
                self.get_logger().info(f'✅ End-effector frame detected: {frame}')
                
                pos = transform.transform.translation
                self.get_logger().info(f'📍 Current position: [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]')
                return
                
            except Exception:
                continue
        
        if not hasattr(self, '_manual_request_sent'):
            self.get_logger().warn('⚠️  Could not auto-detect end-effector frame')
            self.get_logger().warn('Check: ros2 run tf2_tools view_frames')
            self._manual_request_sent = True
    
    def precision_command_callback(self, msg):
        """정밀 명령 처리"""
        command = msg.data.lower().strip()
        
        if command == 'start':
            if self.end_effector_frame:
                self.start_precision_experiment()
            else:
                self.get_logger().error('End-effector frame not detected!')
                
        elif command == 'collect':
            if not self.collecting and self.end_effector_frame:
                self.start_manual_collection()
            else:
                self.get_logger().warn('Already collecting or frame not detected!')
                
        elif command == 'stop':
            if self.collecting:
                self.stop_collection()
            else:
                self.get_logger().warn('Not collecting!')
    
    def start_precision_experiment(self):
        """정밀 실험 시작"""
        self.get_logger().info('🚀 Starting Precision Square Experiment')
        self.experiment_active = True
        self.current_waypoint_index = 0
        self.experiment_log = {
            'start_time': time.time(),
            'waypoints': [],
            'overall_stats': {}
        }
        
        # 실험을 별도 스레드에서 실행
        experiment_thread = threading.Thread(target=self.execute_precision_experiment)
        experiment_thread.daemon = True
        experiment_thread.start()
    
    def start_manual_collection(self):
        """수동 데이터 수집 시작"""
        self.collecting = True
        self.start_time = time.time()
        self.all_positions = []
        self.timestamps = []
        
        self.data_timer = self.create_timer(1.0/self.monitoring_freq, self.collect_tf_data)
        
        self.get_logger().info('🎬 Manual TF data collection started!')
        self.publish_status('Manual collection started')
    
    def stop_collection(self):
        """데이터 수집 종료"""
        self.collecting = False
        
        if hasattr(self, 'data_timer') and self.data_timer:
            self.data_timer.cancel()
            self.data_timer = None
        
        self.get_logger().info(f'🛑 Collection stopped - {len(self.all_positions)} points')
        
        if len(self.all_positions) > 50:
            self.create_precision_visualization()
        else:
            self.get_logger().error('Insufficient data for visualization')
    
    def execute_precision_experiment(self):
        """정밀 실험 실행"""
        print(f"\n🎯 PRECISION SQUARE EXPERIMENT")
        print(f"="*60)
        print(f"📊 Settings:")
        print(f"   Position Tolerance: {self.position_tolerance*1000:.1f}mm")
        print(f"   Settle Tolerance: {self.settle_tolerance*1000:.1f}mm")  
        print(f"   Settle Time: {self.settle_time:.1f}s")
        print(f"   Max Wait Time: {self.max_wait_time:.1f}s")
        print(f"   Retry Attempts: {self.retry_attempts}")
        
        # 데이터 수집 시작
        self.start_manual_collection()
        
        total_success = 0
        
        for i, waypoint in enumerate(self.precision_square):
            self.current_waypoint_index = i
            corner_name = f"Corner {i+1}" if i < 4 else "Return Start"
            
            print(f"\n📍 {corner_name}: [{waypoint[0]:.4f}, {waypoint[1]:.4f}, {waypoint[2]:.4f}]")
            self.publish_status(f"Moving to {corner_name}")
            
            # 재시도 로직
            success = False
            for attempt in range(self.retry_attempts + 1):
                if attempt > 0:
                    print(f"🔄 Retry attempt {attempt}/{self.retry_attempts}")
                    self.publish_status(f"{corner_name} - Retry {attempt}")
                
                waypoint_result = self.execute_precision_waypoint(waypoint, corner_name, attempt)
                
                if waypoint_result['success']:
                    success = True
                    total_success += 1
                    print(f"✅ {corner_name} - SUCCESS!")
                    print(f"   Final Error: {waypoint_result['final_error_mm']:.2f}mm")
                    print(f"   Settle Error: {waypoint_result['settle_error_mm']:.2f}mm")
                    print(f"   Total Time: {waypoint_result['total_time']:.1f}s")
                    break
                else:
                    print(f"❌ {corner_name} - Attempt {attempt+1} FAILED")
                    print(f"   Final Error: {waypoint_result['final_error_mm']:.2f}mm")
                    if attempt < self.retry_attempts:
                        time.sleep(1.0)  # 재시도 전 대기
            
            # 로그 기록
            self.experiment_log['waypoints'].append(waypoint_result)
            
            if not success:
                print(f"💥 {corner_name} - ALL ATTEMPTS FAILED!")
            
            # 다음 웨이포인트로 이동 전 잠시 대기
            time.sleep(0.5)
        
        # 실험 완료
        self.experiment_active = False
        self.stop_collection()
        
        # 최종 보고서
        success_rate = (total_success / len(self.precision_square)) * 100
        total_experiment_time = time.time() - self.experiment_log['start_time']
        
        print(f"\n🎉 PRECISION EXPERIMENT COMPLETED!")
        print(f"   Success Rate: {total_success}/{len(self.precision_square)} ({success_rate:.1f}%)")
        print(f"   Total Time: {total_experiment_time:.1f} seconds")
        
        self.experiment_log['overall_stats'] = {
            'success_rate': success_rate,
            'total_time': total_experiment_time,
            'successful_waypoints': total_success
        }
        
        self.save_experiment_log()
    
    def execute_precision_waypoint(self, target_position, corner_name, attempt):
        """정밀 웨이포인트 실행"""
        start_time = time.time()
        
        # 1단계: 목표 위치로 명령 전송
        success = self.send_precision_command(target_position)
        if not success:
            return {
                'corner': corner_name,
                'attempt': attempt,
                'target': target_position,
                'success': False,
                'total_time': time.time() - start_time,
                'final_error_mm': float('inf'),
                'error_reason': 'Command send failed'
            }
        
        # 2단계: 정밀 도달 모니터링
        return self.monitor_precision_arrival(target_position, corner_name, start_time, attempt)
    
    def send_precision_command(self, position):
        """정밀 명령 전송"""
        try:
            cmd = [
                'ros2', 'topic', 'pub', '--once', '/cartesian_command',
                'geometry_msgs/PointStamped',
                f'{{header: {{frame_id: "{self.base_frame}"}}, point: {{x: {position[0]:.6f}, y: {position[1]:.6f}, z: {position[2]:.6f}}}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
            
        except Exception:
            return False
    
    def monitor_precision_arrival(self, target_position, corner_name, start_time, attempt):
        """정밀 도달 모니터링"""
        errors = []
        settle_errors = []
        settle_start = None
        positions = []
        
        target_array = np.array(target_position)
        
        print(f"⏳ Monitoring arrival (tolerance: {self.position_tolerance*1000:.1f}mm)...")
        
        while (time.time() - start_time) < self.max_wait_time:
            try:
                current_pos = self.get_current_position()
                if current_pos is None:
                    time.sleep(0.02)
                    continue
                
                current_array = np.array(current_pos)
                positions.append(current_pos)
                
                # 오차 계산
                error = np.linalg.norm(current_array - target_array)
                error_mm = error * 1000
                errors.append(error_mm)
                
                # 진행 상황 출력 (1초마다)
                if len(errors) % self.monitoring_freq == 0:
                    elapsed = time.time() - start_time
                    print(f"📏 {elapsed:.1f}s - Error: {error_mm:.2f}mm")
                
                # 허용 오차 내 도달 확인
                if error <= self.position_tolerance:
                    if settle_start is None:
                        settle_start = time.time()
                        print(f"🎯 Reached tolerance! Settling for {self.settle_time}s...")
                    
                    # 안정화 기간 동안 더 엄격한 오차 확인
                    settle_error = error * 1000
                    settle_errors.append(settle_error)
                    
                    # 안정화 완료 확인
                    if (time.time() - settle_start) >= self.settle_time:
                        # 안정화 기간 동안의 평균 오차 확인
                        avg_settle_error = np.mean(settle_errors[-int(self.settle_time * self.monitoring_freq):])
                        
                        if avg_settle_error <= (self.settle_tolerance * 1000):
                            # 성공!
                            total_time = time.time() - start_time
                            final_pos = self.get_current_position()
                            final_error = np.linalg.norm(np.array(final_pos) - target_array) * 1000 if final_pos else error_mm
                            
                            return {
                                'corner': corner_name,
                                'attempt': attempt,
                                'target': target_position,
                                'final_position': final_pos,
                                'success': True,
                                'total_time': total_time,
                                'final_error_mm': final_error,
                                'settle_error_mm': avg_settle_error,
                                'min_error_mm': np.min(errors),
                                'avg_error_mm': np.mean(errors),
                                'error_samples': len(errors),
                                'positions': positions
                            }
                        else:
                            # 안정화 실패 - 다시 시작
                            settle_start = None
                            settle_errors = []
                            print(f"⚠️  Settle failed (avg: {avg_settle_error:.2f}mm), retrying...")
                
                else:
                    # 허용 오차를 벗어나면 안정화 다시 시작
                    settle_start = None
                    settle_errors = []
                
                time.sleep(1.0 / self.monitoring_freq)
                
            except Exception as e:
                print(f"⚠️  Monitoring error: {e}")
                time.sleep(0.1)
        
        # 타임아웃
        total_time = time.time() - start_time
        final_pos = self.get_current_position()
        final_error = np.linalg.norm(np.array(final_pos) - target_array) * 1000 if final_pos else float('inf')
        
        return {
            'corner': corner_name,
            'attempt': attempt,
            'target': target_position,
            'final_position': final_pos,
            'success': False,
            'total_time': total_time,
            'final_error_mm': final_error,
            'settle_error_mm': float('inf'),
            'min_error_mm': np.min(errors) if errors else float('inf'),
            'avg_error_mm': np.mean(errors) if errors else float('inf'),
            'error_samples': len(errors),
            'error_reason': 'Timeout',
            'positions': positions
        }
    
    def collect_tf_data(self):
        """TF 데이터 수집 (고주파수)"""
        if not self.collecting or not self.end_effector_frame:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_frame, 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
            
            pos = transform.transform.translation
            position = [pos.x, pos.y, pos.z]
            
            self.all_positions.append(position)
            self.timestamps.append(time.time() - self.start_time)
            
            # 진행 상황 출력 (덜 빈번하게)
            if len(self.all_positions) % (self.monitoring_freq * 5) == 0:  # 5초마다
                self.get_logger().info(f'📈 {len(self.all_positions)} precision points collected')
        
        except Exception as e:
            pass  # 고주파수 수집에서는 에러 무시
    
    def get_current_position(self):
        """현재 엔드이펙터 위치 가져오기 (정밀)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
            
            pos = transform.transform.translation
            return [pos.x, pos.y, pos.z]
            
        except Exception:
            return None
    
    def publish_status(self, message):
        """상태 메시지 발행"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
    
    def create_precision_visualization(self):
        """정밀 시각화 생성"""
        self.get_logger().info('📊 Creating precision visualization...')
        
        actual_path = np.array(self.all_positions)
        timestamps = np.array(self.timestamps)
        planned_square = np.array(self.precision_square)
        
        # 4개 서브플롯
        fig = plt.figure(figsize=(20, 12))
        
        # Plot 1: Planned vs Actual Path (2D XY)
        ax1 = fig.add_subplot(221)
        
        ax1.plot(planned_square[:, 0], planned_square[:, 1], 'b-o', 
                linewidth=3, markersize=10, label='Planned Path', alpha=0.8)
        ax1.plot(actual_path[:, 0], actual_path[:, 1], 'r-', 
                linewidth=1, label='Actual Path (TF)', alpha=0.7)
        
        # 웨이포인트별 도달 지점 표시
        for i, waypoint_log in enumerate(self.experiment_log.get('waypoints', [])):
            if waypoint_log['success'] and waypoint_log['final_position']:
                final_pos = waypoint_log['final_position']
                color = 'green' if waypoint_log['final_error_mm'] < 2.0 else 'orange'
                ax1.plot(final_pos[0], final_pos[1], 'o', color=color, markersize=8,
                        label=f"Final {i+1}" if i == 0 else "")
        
        # 모서리 번호
        for i, corner in enumerate(planned_square[:-1]):
            ax1.annotate(f'{i+1}', (corner[0], corner[1]), xytext=(5, 5), 
                        textcoords='offset points', fontsize=12, color='blue', weight='bold')
        
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('Precision Path Comparison (XY View)', fontsize=14, weight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: 3D Trajectory
        ax2 = fig.add_subplot(222, projection='3d')
        
        ax2.plot(planned_square[:, 0], planned_square[:, 1], planned_square[:, 2],
                'b-o', linewidth=3, markersize=8, label='Planned Path')
        ax2.plot(actual_path[:, 0], actual_path[:, 1], actual_path[:, 2],
                'r-', linewidth=1, label='Actual Path', alpha=0.7)
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('3D Precision Trajectory')
        ax2.legend()
        
        # Plot 3: Waypoint Accuracy Analysis
        ax3 = fig.add_subplot(223)
        
        if self.experiment_log.get('waypoints'):
            waypoint_errors = []
            waypoint_labels = []
            colors = []
            
            for i, wp in enumerate(self.experiment_log['waypoints']):
                error = wp['final_error_mm']
                waypoint_errors.append(error)
                waypoint_labels.append(f"C{i+1}")
                
                if wp['success']:
                    colors.append('green' if error < 2.0 else 'orange')
                else:
                    colors.append('red')
            
            bars = ax3.bar(range(len(waypoint_errors)), waypoint_errors, color=colors, alpha=0.7)
            
            # 값 표시
            for bar, error in zip(bars, waypoint_errors):
                height = bar.get_height()
                ax3.text(bar.get_x() + bar.get_width()/2, height + 0.1,
                        f'{error:.1f}', ha='center', va='bottom', fontsize=9)
            
            ax3.set_xlabel('Waypoint')
            ax3.set_ylabel('Final Error (mm)')
            ax3.set_title('Waypoint Precision Analysis')
            ax3.set_xticks(range(len(waypoint_labels)))
            ax3.set_xticklabels(waypoint_labels)
            ax3.grid(True, alpha=0.3)
            
            # 허용 오차 라인
            ax3.axhline(y=self.position_tolerance*1000, color='red', linestyle='--', 
                       label=f'Target Tolerance: {self.position_tolerance*1000:.1f}mm')
            ax3.legend()
        
        # Plot 4: Error Distribution
        ax4 = fig.add_subplot(224)
        
        if self.experiment_log.get('waypoints'):
            all_errors = []
            for wp in self.experiment_log['waypoints']:
                if wp['success']:
                    all_errors.append(wp['final_error_mm'])
            
            if all_errors:
                ax4.hist(all_errors, bins=20, alpha=0.7, color='blue', edgecolor='black')
                ax4.axvline(x=np.mean(all_errors), color='red', linestyle='--', 
                           label=f'Mean: {np.mean(all_errors):.2f}mm')
                ax4.axvline(x=self.position_tolerance*1000, color='green', linestyle='--',
                           label=f'Target: {self.position_tolerance*1000:.1f}mm')
                
                ax4.set_xlabel('Final Error (mm)')
                ax4.set_ylabel('Frequency')
                ax4.set_title('Error Distribution')
                ax4.legend()
                ax4.grid(True, alpha=0.3)
        
        # 파일 저장
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'precision_experiment_{timestamp}'
        
        plt.tight_layout()
        plt.savefig(f'{filename}.png', dpi=300, bbox_inches='tight')
        
        # 통계 출력
        self.print_precision_statistics(timestamp)
        
        plt.show()
        self.get_logger().info(f'✅ Precision visualization complete! {filename}.png')
    
    def print_precision_statistics(self, timestamp):
        """정밀 통계 출력"""
        if not self.experiment_log.get('waypoints'):
            return
        
        successful_waypoints = [wp for wp in self.experiment_log['waypoints'] if wp['success']]
        
        print(f"\n📊 PRECISION EXPERIMENT STATISTICS")
        print(f"="*60)
        print(f"🎯 Overall Performance:")
        print(f"   Success Rate: {len(successful_waypoints)}/{len(self.experiment_log['waypoints'])}")
        print(f"   Total Time: {self.experiment_log['overall_stats']['total_time']:.1f}s")
        print(f"   Data Points Collected: {len(self.all_positions)}")
        
        if successful_waypoints:
            final_errors = [wp['final_error_mm'] for wp in successful_waypoints]
            settle_errors = [wp['settle_error_mm'] for wp in successful_waypoints]
            
            print(f"\n📏 Precision Analysis:")
            print(f"   Best Final Error: {np.min(final_errors):.2f}mm")
            print(f"   Worst Final Error: {np.max(final_errors):.2f}mm")
            print(f"   Average Final Error: {np.mean(final_errors):.2f}mm")
            print(f"   Std Dev: {np.std(final_errors):.2f}mm")
            print(f"   Average Settle Error: {np.mean(settle_errors):.2f}mm")
            
            # 허용 오차 내 도달률
            within_tolerance = sum(1 for err in final_errors if err <= self.position_tolerance*1000)
            tolerance_rate = (within_tolerance / len(final_errors)) * 100
            print(f"   Within {self.position_tolerance*1000:.1f}mm: {within_tolerance}/{len(final_errors)} ({tolerance_rate:.1f}%)")
        
        print(f"="*60)
    
    def save_experiment_log(self):
        """실험 로그 저장"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'precision_experiment_log_{timestamp}.json'
        
        log_data = {
            'experiment_info': {
                'timestamp': timestamp,
                'settings': {
                    'position_tolerance_mm': self.position_tolerance * 1000,
                    'settle_tolerance_mm': self.settle_tolerance * 1000,
                    'settle_time': self.settle_time,
                    'max_wait_time': self.max_wait_time,
                    'monitoring_frequency': self.monitoring_freq,
                    'retry_attempts': self.retry_attempts
                }
            },
            'experiment_log': self.experiment_log,
            'tf_data': {
                'timestamps': self.timestamps,
                'positions': self.all_positions,
                'base_frame': self.base_frame,
                'end_effector_frame': self.end_effector_frame
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        self.get_logger().info(f'📄 Experiment log saved: {filename}')


def main():
    rclpy.init()
    
    try:
        visualizer = PrecisionTFVisualizer()
        
        print("\n🎯 Precision TF-based Visualizer Ready!")
        print("\n🔧 Enhanced Features:")
        print("   - 1mm position tolerance")
        print("   - 0.5mm settle tolerance")
        print("   - 50Hz precision monitoring")
        print("   - 2-second stabilization time")
        print("   - Auto-retry on failure")
        print("   - Detailed error analysis")
        print("   - Real-time progress tracking")
        print("\n🚀 Commands:")
        print("   Precision Test: ros2 topic pub --once /precision_command std_msgs/String \"data: start\"")
        print("   Manual Collection: ros2 topic pub --once /precision_command std_msgs/String \"data: collect\"")
        print("   Stop Collection: ros2 topic pub --once /precision_command std_msgs/String \"data: stop\"")
        print("\n📊 Monitor Status:")
        print("   ros2 topic echo /precision_status")
        print("\n⚙️  Settings can be adjusted in the code:")
        print("   - position_tolerance: Currently 1mm")
        print("   - settle_tolerance: Currently 0.5mm")
        print("   - monitoring_freq: Currently 50Hz")
        print("   - retry_attempts: Currently 2")
        
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("Terminated")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()