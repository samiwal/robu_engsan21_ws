from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    node_ledstrip_sub = Node(
        package="robu",
        executable="led_strip_sub",
        output="screen",
        # remappings =
        # parameters =
        # arguments
    )

    exec_ledstrip_pub = ExecuteProcess(
        cmd=['gnome-terminal','--','bash', '-c', "'source /home/samuel/work/robu_bhme21_ws/install/setup.bash && ros2 run robu led_strip_pub'"],
        shell = True
    )

    node_ledstrip_pub = Node(
        package="robu",
        executable="led_strip_sub",
        output="screen",
        # remappings =
        # parameters =
        # arguments =

    )
    ld = LaunchDescription()
    ld.add_action(node_ledstrip_sub)
    ld.add_action(exec_ledstrip_pub)

    return ld