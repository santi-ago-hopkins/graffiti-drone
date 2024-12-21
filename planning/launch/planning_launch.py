from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planner',
            executable='planner_node', 
            name='planner',
            output='screen',
        )
    ])
