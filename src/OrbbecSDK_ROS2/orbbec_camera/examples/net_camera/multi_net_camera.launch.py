from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "femto_mega.launch.py")  # If you are using Gemini 335Le, replace femto_mega.launch.py ​​with gemini_330_series.launch.py
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "net_device_ip": "192.168.1.10",
            "net_device_port": "8090",
            "sync_mode": "standalone",
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "femto_mega.launch.py")  # If you are using Gemini 335Le, replace femto_mega.launch.py ​​with gemini_330_series.launch.py
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "net_device_ip": "192.168.1.11",
            "net_device_port": "8090",
            "sync_mode": "standalone",
        }.items(),
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [
            TimerAction(period=0.0, actions=[GroupAction([launch1_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch2_include])]),
        ]
    )

    return ld
