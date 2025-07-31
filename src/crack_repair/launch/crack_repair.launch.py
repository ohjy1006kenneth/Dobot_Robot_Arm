import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    femto_mega = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'), 'launch', 'femto_mega.launch.py'
            )
        )
    )

    ranger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ranger_bringup'), 'launch', 'ranger.launch.xml'
            )
        )
    )

    dobot_cr20 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cr_robot_ros2'), 'launch', 'dobot_bringup_ros2.launch.py'
            )
        )
    )

    # Your nodes to launch
    image_processing_node = Node(
        package='image_processing',
        executable='line_follower',
        name='line_follower',
        output='screen'
    )

    robot_car_node = Node(
        package='ranger_control',
        executable='line_follower_controller',
        name='line_follower_controller',
        output='screen'
    )

    robot_arm_node = Node(
        package='robot_arm',
        executable='script_runner',
        name='script_runner',
        output='screen'
    )

    return LaunchDescription([
        femto_mega,
        ranger,
        dobot_cr20,
        image_processing_node,
        robot_car_node,
        robot_arm_node,
    ])
