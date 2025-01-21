# sfbot_can

   ros2_control 프레임워크를 이용하여 SFBot 개발을 위한 연습 과정입니다. 2자유도를 가진 모터를 제어합니다.  

   개발자 : foody-j  


Hardware Interface Class : SfBotSystemHardware  
namespace : sfbot_can   

CAN Communication driver header : motor_can_driver.hpp  
Motor Data Structure header : motor_data.hpp  

# 변경사항
   2025.01.21  
   motor_data.hpp 파일 수정  
      MotorData 구조체에 motor_id 추가  
      
      struct MotorData {
         uint8_t motor_id{0};     // 모터 ID (1~6)
      }

      static constexpr size_t MAX_MOTORS = 6;
    
      MotorData& getMotorData(uint8_t motor_id) {
         if (motor_id < 1 || motor_id > MAX_MOTORS) {
               throw std::runtime_error("Invalid motor ID");
         }
         motor_data_[motor_id - 1].motor_id = motor_id; // ID 자동 설정
         return motor_data_[motor_id - 1];
      }

      void updateMotorData(uint8_t motor_id, const MotorData& data) {
         if (motor_id < 1 || motor_id > MAX_MOTORS) {
               throw std::runtime_error("Invalid motor ID");
         }
         motor_data_[motor_id - 1] = data;
         motor_data_[motor_id - 1].motor_id = motor_id; // ID 보장
      }

      void reset() {
         for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
               motor_data_[i] = MotorData{};
               motor_data_[i].motor_id = i + 1; // 리셋 시에도 ID 유지
         }
      }

