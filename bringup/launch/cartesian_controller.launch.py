from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 실행 인자 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="your_robot_description_package",
            description="Robot description package name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value="your_robot.urdf.xacro",
            description="XACRO file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config_file",
            default_value="controller_config.yaml",
            description="Controller configuration file",
        )
    )

    # 인자 가져오기
    robot_description_package = LaunchConfiguration("robot_description_package")
    xacro_file = LaunchConfiguration("xacro_file")
    controller_config_file = LaunchConfiguration("controller_config_file")

    # 로봇 설명 생성
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "urdf", xacro_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 컨트롤러 매니저 노드
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "config", controller_config_file]
            ),
        ],
        output="screen",
    )

    # 컨트롤러 로더 노드
    cartesian_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 카테시안 명령 발행 예제 노드
    cartesian_command_publisher = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="cartesian_command_publisher",
        remappings=[("/chatter", "/cartesian_controller/target_pose")],
        parameters=[
            {"use_sim_time": True},
            {"publish_frequency": 1.0},
        ],
        output="screen",
    )

    # RViz 시작
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare(robot_description_package), "config", "cartesian_control.rviz"]),
        ],
    )

    # Launch description 반환
    return LaunchDescription(
        declared_arguments + 
        [
            controller_manager_node,
            cartesian_controller_spawner,
            cartesian_command_publisher,
            rviz_node,
        ]
    )