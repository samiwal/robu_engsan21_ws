import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'robu'
    #Niemals absolute Pfade verwenden (nur zum Testen!)
    robu_shared_dir = get_package_share_directory(pkg_name)
    robot_xacro_file = os.path.join(robu_shared_dir, 'urdf', 'test_robot.xacro')

    robot_description = xacro.process_file(robot_xacro_file).toxml()


    node_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
        'use_sim_time': True}],
        output='screen'
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )

    # Run the node
    return LaunchDescription([
        node_state_publisher,
        node_rviz
    ])