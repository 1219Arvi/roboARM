import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_arguments = {
        "use_fake_hardware": "false",
        "dof": "7",
    }

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("fixy_config"),
            "config",
            "fixy.urdf.xacro"
        ]),
        " ",
        "use_fake_hardware:=", launch_arguments["use_fake_hardware"], " ",
        "dof:=", launch_arguments["dof"]
    ])

    robot_description = {"robot_description": robot_description_content}

    # Modified planning pipeline configuration to match working example
    moveit_config = (
        MoveItConfigsBuilder("rhea", package_name="fixy_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines("ompl", ["ompl"])  # Exact same format as working example
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "ros2_controllers.yaml"
    )

    planning_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "planning_pipeline.yaml"
    )

    ompl_planning_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "ompl_planning.yaml"
    )

    arm_controller_params_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "arm_controller_params.yaml"  # flat YAML for spawner without controller_manager: root
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("fixy_config"),
            "config",
            "moveit.rviz"
        ]),
        description="RViz configuration file"
    )
    rviz_config = LaunchConfiguration("rviz_config")


    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ros2_controllers_path],
        # arguments=['--ros-args', '--log-level', 'debug'],
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c", "/controller_manager",
            "--param-file", arm_controller_params_path  # pass flat YAML here
        ],
    )

    joint_state_broadcaster_event = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    arm_controller_event = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        rviz_config_arg,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_event,
        arm_controller_event,
        run_move_group_node,
        rviz_node,
    ])
