import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the package and URDF
    pkg_bringup = get_package_share_directory('surface_reconstruction')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'surface_reconstruction.urdf')
    rslidar_config_file = os.path.join(pkg_bringup, 'config', 'rslidar_config.yaml')
    overlap_roi_mode = LaunchConfiguration('overlap_roi_mode')
    overlap_roi_axis = LaunchConfiguration('overlap_roi_axis')
    overlap_roi_ratio = LaunchConfiguration('overlap_roi_ratio')

    # Robot State Publisher (Reads URDF and publishes TF)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
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
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        remappings=[
            ('cloud_in', '/rslidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'rslidar',
            'transform_tolerance': 0.2,
            'min_height': -0.5,
            'max_height': 1.0,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
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
            'log_level': 'warn',
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
            'log_level': 'warn',
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
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{'config_path': rslidar_config_file}]
    )

    # Laser scanner pipeline (driver + accumulator + coordinator)
    laser_scanner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('laser_scanner'), 'launch', 'scanner_system.launch.py')
        ),
        launch_arguments={
            # Use C++ nodes by default (recommended).
            'use_cpp': 'true',
            'launch_script_runner': 'true',

            # Frames must match Nav2 / base TF tree.
            'fixed_frame': 'base_link',
            'odom_frame': 'odom',
            'output_directory': os.path.join(os.path.expanduser('~'), 'Dobot_Robot_Arm', 'scans'),
            'overlap_roi_mode': overlap_roi_mode,
            'overlap_roi_axis': overlap_roi_axis,
            'overlap_roi_ratio': overlap_roi_ratio,

            # If your URDF already defines/publishes laser_frame, keep this false.
            # If scan_accumulator logs TF lookup failures for base_link<->laser_frame,
            # set this true and fill xyz/rpy.
            'publish_static_laser_tf': 'false',
            'laser_tf_xyz': '0 0 0',
            'laser_tf_rpy': '0 0 0',
        }.items()
    )

    # When Nav2 reaches a goal, publish /start_repair (std_msgs/Bool)
    nav2_arrival_trigger = Node(
        package='surface_reconstruction',
        executable='nav2_arrival_trigger',
        name='nav2_arrival_trigger',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'min_start_repair_subscribers': 2,
            'trigger_publish_count': 5,
            'trigger_publish_period_ms': 250,
            'subscriber_wait_timeout_ms': 5000,
            'start_repair_cooldown_ms': 3000,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'overlap_roi_mode',
            default_value='dynamic_z',
            choices=['dynamic_z', 'tf_only', 'fixed'],
            description='Scan accumulation mode: dynamic_z, tf_only, or fixed.',
        ),
        DeclareLaunchArgument('overlap_roi_axis', default_value='y'),
        DeclareLaunchArgument('overlap_roi_ratio', default_value='0.30'),
        robot_state_publisher,
        pc_to_laserscan,
        ranger_launch,
        dobot_launch,
        rslidar_node,
        laser_scanner_launch,
        nav2_localization_launch,
        nav2_navigation_launch,
        nav2_arrival_trigger,
    ])
