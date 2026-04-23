import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the package and URDF
    pkg_bringup = get_package_share_directory('surface_reconstruction')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'surface_reconstruction.urdf')
    rslidar_config_file = os.path.join(pkg_bringup, 'config', 'rslidar_config.yaml')

    # Robot State Publisher (Reads URDF and publishes TF)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=['robot_description'],
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
            'ignore_timestamp': True,
        }],
        remappings=[('/joint_states', '/joint_states_robot')]
    )

    # Dobot Arm
    dobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cr_robot_ros2'), 'launch', 'dobot_bringup_ros2.launch.py')
        )
    )

    # Pointcloud to Laserscan
    pc_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/rslidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'rslidar',
            'transform_tolerance': 0.2,
            # Tight Z slice to avoid floor/ceiling/robot body hits.
            # If the LiDAR is upside down, adjust these signs after checking the pointcloud Z axis.
            'min_height': -0.10,
            'max_height': 0.10,
            # Use full 360deg scan unless you intentionally only want a front-facing scan.
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'use_inf': True,
            'range_min': 0.15,
            'use_sim_time': False,
        }]
    )

    # Nav2 Localization (map_server + amcl) - publishes map->odom when active
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            'use_composition': 'False',
            'map': os.path.join(pkg_bringup, 'map', 'CATS_Lab.yaml'),
        }.items()
    )

    # Nav2 Navigation stack
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            'use_composition': 'False',
        }.items()
    )

    # Ranger Base Bringup
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ranger_base'), 'launch', 'ranger.launch.py')
        ),
        launch_arguments={
            'publish_odom_tf': 'true'
        }.items()
    )

    # RoboSense LiDAR Node
    rslidar_node = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': rslidar_config_file}]
    )

    return LaunchDescription([
        robot_state_publisher,
        pc_to_laserscan,
        ranger_launch,
        dobot_launch,
        rslidar_node,
        nav2_localization_launch,
        nav2_navigation_launch,
    ])