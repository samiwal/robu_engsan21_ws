from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    node_ledstrip_sub = Node(
        package="robu",
        executable="ledstrip_sub",
        output="screen",
        # emulate_tty=True
        # remappings=
        # parameters=
        # arguments=
    )

    exec_ledstrip_pub = ExecuteProcess(
        
        cmd=['gnome-terminal', '--', 'bash', '-c', "'source /home/samuel/work/robu/robu_bhme21_ws/install/setup.bash && ros2 run robu ledstrip_pub'"],
        # cmd=['terminator', 
        #      '-e', '\'bash -c "source /opt/ros/humble/setup.bash; source /home/robu/work/ROBU/robu_bhme21_ws/install/setup.bash; ros2 run robu plf01_pub; exec bash"\''],
        shell=True,
        # cmd="ros2 run robu plf01_pub",
        # pty=True  #geht nur mit ros2 galactic oder Neuer!!
    )
    # node_ledstrip_pub = Node(
    #     package="robu",
    #     executable="plf01_pub",
    #     output="screen",
    #     emulate_tty=True,
    #     prefix="gnome-terminal -- bash -c 'source /home/robu/work/ROBU/robu_bhme21_ws/install/setup.bash && ros2 run robu plf01_pub'",
    #     #arguments=["'"]
    # )

    ld = LaunchDescription()
    ld.add_action(node_ledstrip_sub)
    ld.add_action(exec_ledstrip_pub)

    #ld.add_action(node_ledstrip_pub)


    return ld