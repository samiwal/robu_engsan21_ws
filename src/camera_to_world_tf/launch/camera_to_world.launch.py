from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'camera_to_world_tf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name), 'urdf','arm_with_camera.urdf'
    )

    node_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {"robot_description": open(urdf_path).read()}
        ]
    )

    node_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    node_camera_to_world = Node(
        package='camera_to_world_tf',
        executable='camera_to_world',
        name='knoten_name',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(node_publisher)
    ld.add_action(node_gui)
    ld.add_action(node_rviz)
    ld.add_action(node_camera_to_world)
    
    return ld