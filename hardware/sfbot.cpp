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

// 이전에 정의한 RROBT 헤더 파일 포함
#include "sfbot_can/sfbot.hpp"

// CAN 통신 드라이버 헤더 추가
#include "sfbot_can/motor_can_driver.hpp" //CAN 통신 클래스 정의된 헤더
#include "sfbot_can/motor_data.hpp" // 모터 데이터 객체 정의 헤더

// 필요한 표준 라이브러리 헤더들 포함
#include <chrono> // 시간 관련 기능
#include <cmath>  // 수학 함수
#include <iomanip>  // 입출력 포맷팅
#include <limits> //수치 한계 값
#include <memory> // 스마트 포인터
#include <sstream>  // 문자열 스트림
#include <string> // 문자열 처리
#include <vector> // 벡터 컨테이너

// ROS 2 관련 헤더 포함
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sfbot_can
{
  // 하드웨어 초기화 함수 구현 ..
hardware_interface::CallbackReturn SfBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (  // 부모 클래스의 초기화가 실패하면 에러 반환
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // 하드웨어 파라미터 설정
  /*
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];  // 왼쪽 바퀴 이름 설정
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  */
  // 일단 이 부분 패스해도 문제 없을 듯. 이미 CAN 에서 초기화 함.
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  // wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  // wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
std::vector<hardware_interface::StateInterface> SfBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &pos_[0]));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //"ak70-10-v1_1_continuous", hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}
// 위치 명령 인터페이스
std::vector<hardware_interface::CommandInterface> SfBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &cmd_[0]));
  //command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}*/
// 멤버 함수로 정확히 정의
std::vector<hardware_interface::StateInterface> SfBotSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &pos_[0]));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SfBotSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &cmd_[0]));
    return command_interfaces;
}

hardware_interface::CallbackReturn SfBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (can_driver.connected())
  {
    can_driver.disconnect();
  }
  can_driver.connect("can0", 1000000);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SfBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (can_driver.connected())
  {
    can_driver.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SfBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    
    if (!can_driver.connected()) {
        can_driver.connect("can0", 1000000);
    }
    
    // 원점 설정 전에 연결 확인
    if (!can_driver.connected()) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to CAN bus");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (can_driver.initialize_motor_origin(1)) {
        RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // 여기에 속도와 가속도 초기화 추가
        velocity_ = 100.0f;  // RPM
        acceleration_ = 100.0f;  // RPM/s
        can_driver.write_position_velocity(1, 0.0, velocity_, acceleration_);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to initialize motor origin");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SfBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  //comms_.disconnect(); // 연결 끊기
  can_driver.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SfBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    if (!can_driver.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  for (uint8_t i = 1; i < 3; i++)  // 모터 1번과 2번의 데이터를 가져옴
  {
      motor_data = can_driver.getMotorData(i);
      pos_[i-1] = motor_data.position;
      spd_[i-1] = motor_data.speed;
      
      std::cout << std::dec;  // 10진수 모드로 명시적 설정
      std::cout << "Motor " << static_cast<int>(motor_data.motor_id) << ": "
          << "Position: " << pos_[i-1] << "° "
          << "Speed: " << spd_[i-1] << " RPM "
          << "Current: " << motor_data.current << "A "
          << "Temperature: " << static_cast<int>(motor_data.temperature) << "°C "
          << "Error: 0x" << std::hex << static_cast<int>(motor_data.error) 
          << std::dec << std::endl;
  }

  /*
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;*/

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SfBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!can_driver.connected()) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    // radian to degree 변환 추가
      double degree = cmd_[0] * 180.0 / M_PI;  // cmd_[0]는 radian 값을 degree로 변환
      // position-velocity 모드로 명령 전송
      can_driver.write_position_velocity(1, degree, velocity_, acceleration_);
  }
  catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("SfBotSystemHardware"), "Failed to write command: %s", e.what());
      return hardware_interface::return_type::ERROR;
  }
  // write 함수는 일단 비움..
  /*int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);*/

  return hardware_interface::return_type::OK;
}

}  // namespace sfbot_can

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sfbot_can::SfBotSystemHardware, hardware_interface::SystemInterface)
