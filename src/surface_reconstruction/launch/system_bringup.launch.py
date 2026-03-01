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
    map_dir = os.path.join(pkg_bringup, 'map', 'CATS_Lab')
    rslidar_config_file = os.path.join(pkg_bringup, 'config', 'rslidar_config.yaml')
    fastlivo_config_file = os.path.join(pkg_bringup, 'config', 'fast_livo_airy.yaml')

    # Robot State Publisher (Reads URDF and publishes TF)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=['robot_description'],
        parameters=[{'robot_description': robot_desc,
                    'use_sim_time': False,
                    'ignore_timestamp': True
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
            ('cloud_in', '/rslidar_points'), # Raw point cloud from LiDAR
            ('scan', '/scan')                # 2D Output from Nav2
        ],
        parameters=[{
            'target_frame': 'rslidar',
            'transform_tolerance': 0.01,
            'min_height': 0.5,    # Distance to look down 
            'max_height': 2.0,     # Distance to look up 
            'angle_min': -1.5708,  # Look 90 degrees to the left
            'angle_max': 1.5708,   # Look 90 degrees to the right
            'use_inf': True,
            'range_min': 0.4,      # Ignore Zone
        }]
    )
    
    # Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            'use_composition': 'false'
        }.items()
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'publish_odom_tf': 'true',
            'map': map_dir
        }.items()
    )
    
    # Ranger Base Bringup
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ranger_base'), 'launch', 'ranger.launch.py'
            )
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

    # IMU relay: converts g-force -> m/s² for FAST_LIO
    imu_relay_node = Node(
        package='surface_reconstruction',
        executable='imu_g_to_ms2_relay',
        name='imu_g_to_ms2_relay',
        output='screen',
        parameters=[{
            'input_topic':  '/rslidar_imu_data',
            'output_topic': '/rslidar_imu_data_m2',
        }]
    )

    # FAST-LIO Node (kept for fallback — swap fast_livo_node <-> fast_lio_node in LaunchDescription)
    fastlio_config_dir = os.path.join(pkg_bringup, 'config')
    fastlio_config_file = 'fast_lio.yaml'
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_path': fastlio_config_dir,
            'config_file': fastlio_config_file,
            'rviz': 'true'
        }.items()
    )

    # FAST-LIVO2 Node (LiDAR-Inertial only, img_en: 0)
    fast_livo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_livo'), 'launch', 'mapping_airy.launch.py')
        ),
        launch_arguments={
            'airy_params_file': fastlivo_config_file,
            'use_rviz': 'True'
        }.items()
    )

    # Static transform: camera_init -> rslidar
    # camera_init is FAST-LIVO2's fixed world frame; rslidar is the LiDAR sensor frame.
    # This is an identity transform — they start co-located at origin.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_init_to_rslidar',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'rslidar'],
        output='screen'
    )

    return LaunchDescription([
        # robot_state_publisher,
        # pc_to_laserscan,
        # nav2_launch,
        # ranger_launch,
        # dobot_launch,
        static_tf_lidar,
        rslidar_node,
        imu_relay_node,  # Required: accel must be in m/s² (not g-force)
        # fast_lio_node,   # Fallback: uncomment to switch back to FAST-LIO
        fast_livo_node,
    ])