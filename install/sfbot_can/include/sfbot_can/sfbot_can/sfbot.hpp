// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL__SFBOT_HPP_
#define ROS2_CONTROL__SFBOT_HPP_

#include <memory> // 필요한 C++ 표준 라이브러리 헤더 포함
#include <string> // 스마트 포인터 사용을 위한 헤더
#include <vector> // 벡터 컨테이너 사용을 위한 헤더

// ROS 2 하드웨어 인터페이스 관련 헤더 들
#include "hardware_interface/handle.hpp"  // 하드웨어 핸들 정의
#include "hardware_interface/hardware_info.hpp" // 하드웨어 정보 구조체 정의
#include "hardware_interface/system_interface.hpp"  // 시스템 인터페이스 기본 클래스
#include "hardware_interface/types/hardware_interface_return_values.hpp" // 반환값 타입 정의
#include "rclcpp/macros.hpp"  // ROS 2 C++ 매크로 정의
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"  // 생명 주기 노드 정의
#include "rclcpp_lifecycle/state.hpp" // 생명 주기 상태 정의
#include "motor_can_driver_multiple.hpp"
#include "motor_data.hpp"


// ROS 2 컨트롤 데모 예제 1의 네임스페이스 시작
namespace sfbot_can
{
  // RRBot 시스템의 위치 제어만을 위한 하드웨어 인터페이스 클래스
class SfBotSystemHardware : public hardware_interface::SystemInterface
{

  
public:
  // 공유 포인터 정의를 위한 매크로
  RCLCPP_SHARED_PTR_DEFINITIONS(SfBotSystemHardware);
  
  // 하드웨어 초기화 콜백 함수
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  // 하드웨어 설정 콜백 함수
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 클린업 콜백 함수
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 활성화 콜백 함수
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 비활성화 콜백 함수
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 상태 읽기 함수
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  // 하드웨어 명령 쓰기 함수
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation
  CanComms can_driver;  // CAN 통신 객체
  MotorDataManager motor_manager_;
  int hw_start_sec_ = 2;  // 또는 원하는 값으로 설정
  double pos_[6] = {0.0f};
  float spd_[6] = {0.0f};
  double cmd_[6] = {0.0};  // 위치 명령을 저장할 배열
  float velocity_;      // RPM
  float acceleration_;  // RPM/s

  MotorData motor_data;

};

}  // namespace sfbot_can

#endif  // SFBOT_CAN__RRBOT_HPP_
