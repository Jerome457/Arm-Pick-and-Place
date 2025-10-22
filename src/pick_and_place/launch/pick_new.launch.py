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
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", os.path.join(get_package_share_directory("arm_urdf"), "urdf", "object.urdf"),
            "-entity", "cylinder",
            "-x", "0.0", "-y", "-0.6", "-z", "0.05",
            "-R", str(radians(90)),            # roll
            "-P", "0.0", # pitch -> lying along X
            "-Y", "0.0"             # yaw
        ],
        output="screen"
    )

    return LaunchDescription([spawn_mesh_node])