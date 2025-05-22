import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'robu'
    robu_shared_dir = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(robu_shared_dir,'urdf','test_robot.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robot_description_raw,'use_sim_time':True}] # add other parameters here if required
    )

    # node_robot_state_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name="rviz2",
    )


    # Run the node
    return LaunchDescription([
        node_state_publisher,
    #    node_robot_state_gui,
        node_rviz
    ])