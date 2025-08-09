from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    # Launch configuration variables
    gui = LaunchConfiguration("gui")
    rviz_config_file = LaunchConfiguration("rviz_config")
    controllers_file = LaunchConfiguration("controllers_file")
    hardware_file = LaunchConfiguration("hardware_file")

    robot_description_content = Command(
        [
            'xacro',
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('fixy_config'),
                    'config',
                    'fixy.urdf.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # Declare launch args
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable RViz GUI",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("fixy_config"), "config", "moveit.rviz"
            ]),
            description="Path to RViz config file",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("fixy_config"), "config", "ros2_controllers.yaml"
            ]),
            description="Path to ROS2 controller config file"
        ),
        DeclareLaunchArgument(
            "hardware_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("hardware_controller"), "config", "hardware.yaml"
            ]),
            description="Path to hardware interface config file"
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_file],
            output='screen'
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(gui),
        ),

        # Spawner for joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),

        # Spawner for arm controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--param-file", controllers_file],
            output="screen"
        )
    ])
