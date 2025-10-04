from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
import os
import xacro
import yaml
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Define robot description files
    robot_description_file = os.path.join(
        get_package_share_directory("arm_urdf"),
        "urdf",
        "ArmPlate.xacro"
    )

    robot_description_config = xacro.process_file(robot_description_file)
    robot_urdf = robot_description_config.toxml()

    joint_controllers_file = os.path.join(
        get_package_share_directory("arm_urdf_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )


    # Robot description parameter
    # robot_description = Command(["xacro ", robot_urdf])  # Fixed extra space issue

    # MoveIt2 Configuration
    moveit_config = (
        MoveItConfigsBuilder("ArmPlate",package_name="arm_urdf_moveit_config")
        .robot_description(file_path="config/ArmPlate.urdf.xacro")
        .robot_description_semantic(file_path="config/ArmPlate.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


    # RViz Config
    rviz_config_path = os.path.join(
        get_package_share_directory("arm_urdf"),
        "config",
        "display.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
        ]
    )


    with open(joint_controllers_file, "r") as f:
        controllers_yaml = yaml.safe_load(f)


    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers_yaml],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description]
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Move Group Node
    config_dict = moveit_config.to_dict()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # Delayed Start Handlers
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[hand_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    # Launch Description
    ld = LaunchDescription()

    # Add launch actions
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz_node)
    ld.add_action(move_group_node)


    return ld  # Correct return statement
