from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to the individual launch files
    pid_launch_path = os.path.join(get_package_share_directory('pid_package'), 'launch', 'pid_launch.py')
    planning_launch_path = os.path.join(get_package_share_directory('planning_package'), 'launch', 'planning_launch.py')
    microcontroller_launch_path = os.path.join(get_package_share_directory('microcontroller_package'), 'launch', 'microcontroller_launch.py')

    return LaunchDescription([
        # Include PID launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pid_launch_path)
        ),
        # Include Planning launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch_path)
        ),
        # Include Microcontroller launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(microcontroller_launch_path)
        ),
    ])
