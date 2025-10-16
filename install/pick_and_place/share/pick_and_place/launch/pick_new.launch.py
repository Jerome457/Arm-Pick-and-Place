from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
from math import radians,pi
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ArmPlate",package_name="arm_urdf_moveit_config").to_dict()
    urdf_path= os.path.join(get_package_share_directory("arm_urdf"),"urdf","object.xacro")
    # MTC Demo node
    pick_place_demo = Node(
        package="pick_and_place",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
    spawn_mesh_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_mesh_object',
        arguments=[
            '-entity', 'object',        # name of the object in Gazebo
            '-file', urdf_path,         # path to URDF
            '-x', '0.0', '-y', '-0.6', '-z', '0.1',
            '-R', '0', '-P', f'{pi/2}', '-Y', '0'  # 90° pitch rotation
        ],
        output='screen'
    )
    return LaunchDescription([spawn_mesh_node])