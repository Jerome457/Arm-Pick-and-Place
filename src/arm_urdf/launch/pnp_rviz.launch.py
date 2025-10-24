import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    with open(absolute_path, 'r') as file:
        return yaml.safe_load(file)
    
def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("ArmPlate",package_name="arm_urdf_moveit_config")
        .robot_description(file_path="config/ArmPlate.urdf.xacro")
        .robot_description_semantic(file_path="config/ArmPlate.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl",pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )
    # Add sim time globally
    sim_time = {"use_sim_time": True}

    octomap_updater_config = load_yaml('arm_urdf_moveit_config', 'config/sensors_3d.yaml')
    octomap_config = {'octomap_frame': 'world', 
                      'octomap_resolution': 0.01,
                      'max_range': 5.0}

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    gazebo_launch_file = os.path.join(
    get_package_share_directory("gazebo_ros"),
    "launch",
    "gazebo.launch.py"
    )
    # Get the path to the Gazebo empty world file
    empty_world_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'
    )


    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            sim_time,
            octomap_config,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("arm_urdf") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            sim_time,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            sim_time,
        ],
    )
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_launch_file),
    launch_arguments={
        "use_sim_time": "true",
        "debug": "false",
        "gui": "true",
        "paused": "true",
        "world": empty_world_file,
    }.items()
    )
    spawn_the_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "arm_urdf",
            "-topic", "/robot_description",
        ],
        output="screen"
    )
    relay_node= Node(
            package='arm_urdf',
            executable='pointcloud_relay_node',
            name='pointcloud_relay',
            output='screen'
        )

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            spawn_the_robot,
            gazebo,
            relay_node
        ]
        + load_controllers
    )