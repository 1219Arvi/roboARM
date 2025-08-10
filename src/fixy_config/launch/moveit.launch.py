import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def load_yaml(package_name, relative_path):
    yaml_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    launch_arguments = {
        "use_fake_hardware": "false",
        "dof": "7",
    }

    robot_description = {
        "robot_description": Command([
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
    }

    robot_description_semantic = {
        "robot_description_semantic": open(
            os.path.join(
                get_package_share_directory("fixy_config"),
                "config",
                "fixy.srdf"
            ), 'r'
        ).read()
    }

    kinematics_yaml = load_yaml("fixy_config", "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml("fixy_config", "config/ompl_planning.yaml")
    planning_pipeline_yaml = load_yaml("fixy_config", "config/planning_pipeline.yaml")
    moveit_controllers_yaml = load_yaml("fixy_config", "config/moveit_controllers.yaml")
    moveit_controllers_manager_yaml = load_yaml("fixy_config", "config/moveit_controller_manager.yaml")
    joint_limits_yaml = load_yaml("fixy_config", "config/joint_limits.yaml")
    pilz_cartesian_limits_yaml = load_yaml("fixy_config", "config/pilz_cartesian_limits.yaml")

    planning_pipeline_config = {
        "move_group": {
            "planning_pipelines": ["ompl", "chomp", "pilz_industrial_motion_planner"],
            "default_planning_pipeline": "ompl",
        }
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    ros2_controllers_yaml_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "ros2_controllers.yaml"
    )
    arm_controller_params_yaml_path = os.path.join(
        get_package_share_directory("fixy_config"),
        "config",
        "ros2_controllers.yaml"
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
        parameters=[robot_description, ros2_controllers_yaml_path],  # pass YAML path here
    )
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            planning_pipeline_yaml,
            moveit_controllers_yaml,
            moveit_controllers_manager_yaml,
            joint_limits_yaml,  # ADD THIS
            pilz_cartesian_limits_yaml,  # ADD THIS
            planning_pipeline_config,  # ADD THIS
            trajectory_execution,  # ADD THIS
            planning_scene_monitor_parameters,  # ADD THIS
        ],
        arguments=['--ros-args', '--log-level', 'debug'],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            planning_pipeline_yaml,
            moveit_controllers_yaml,
            moveit_controllers_manager_yaml,
            joint_limits_yaml,  # ADD THIS
            pilz_cartesian_limits_yaml,  # ADD THIS
            planning_pipeline_config,  # ADD THIS
            trajectory_execution,  # ADD THIS
            planning_scene_monitor_parameters,  # ADD THIS
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
            "--param-file", arm_controller_params_yaml_path  # path here too
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

    delayed_move_group = TimerAction(
        period=10.0,
        actions=[run_move_group_node]
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_event,
        arm_controller_event,
        delayed_move_group,
        rviz_config_arg,
        rviz_node,
    ])
