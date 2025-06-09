import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Kopiere alle aktuellen Umgebungsvariablen
    env = os.environ.copy()
    
    return LaunchDescription([
        # Starte den Node mit sudo
        ExecuteProcess(
            cmd=[
                "sudo",
                "-E env",
                "\"ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY\"",
                "\"RMW_FASTRTPS_USE_SHM=$RMW_FASTRTPS_USE_SHM\"",
                "\"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\"",
                "\"RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION\"",
                "\"PYTHONPATH=$PYTHONPATH\"",
                "\"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\"",
                "\"PATH=$PATH\"",
                "\"USER=$USER\"",
                "bash -c",
                "'",
                'ros2 run ledctrl ledctrl',
                "'"
            ],
            shell=True,
            output='screen'
        )
    ])
