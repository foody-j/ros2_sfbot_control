cmake_minimum_required(VERSION 3.16)
project(sfbot_can LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  sfbot_can
  SHARED
  hardware/sfbot.my_robot.cpp
)
target_compile_features(sfbot_can PUBLIC cxx_std_17)
target_include_directories(sfbot_can PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/sfbot_can>
)
ament_target_dependencies(
  sfbot_can PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sfbot_can.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/sfbot_can
)

install(
  DIRECTORY description
  DESTINATION share/sfbot_can
)

# STL 파일들 설치
install(DIRECTORY description/urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/description/urdf/meshes/
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/sfbot_can
)
install(
  PROGRAMS scripts/advanced_cartesian_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  scripts/advanced_cartesian_controller.py
  scripts/simple_action_test.py # <<< 이 줄을 추가하세요
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/advanced_cartesian_controller.py
  scripts/simple_action_test.py 
  scripts/direct_command_test.py # <<< 이 줄을 추가
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sfbot_can
  EXPORT export_sfbot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY rviz
  DESTINATION share/sfbot_can
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_1_launch test/test_rrbot_launch.py)
  ament_add_pytest_test(run_example_1_launch_cli_direct test/test_rrbot_launch_cli_direct.py)
endif()


## EXPORTS
ament_export_targets(export_sfbot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
