from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sdk_launch_dir = get_package_share_directory('cr20_moveit')  # your official SDK package name

    return LaunchDescription([
        # Include the original SDK MoveIt launch to bring up robot and MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sdk_launch_dir, 'launch', 'dobot_moveit.launch.py')
            )
        ),
        # Run your controller node alongside
        # Node(
        #     package='robot_arm',  # your package name
        #     executable='controller',
        #     output='screen',
        # )
    ])
