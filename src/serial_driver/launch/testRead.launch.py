import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='serial_driver',
            executable='serial_driver',
            name='serial_driver',
            parameters=[]
        )
    ])