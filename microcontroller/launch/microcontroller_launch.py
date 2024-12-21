from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='microcontroller',
            executable='arduino_connection_node', 
            name='microcontroller',
            output='screen',
        )
    ])
