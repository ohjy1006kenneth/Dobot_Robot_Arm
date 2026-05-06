#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # This launch is intended to be INCLUDED from the main system bringup.
    # The main bringup already runs robot_state_publisher, so we avoid starting
    # another copy here.

    fixed_frame = LaunchConfiguration("fixed_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    output_directory = LaunchConfiguration("output_directory")

    publish_static_laser_tf = LaunchConfiguration("publish_static_laser_tf")
    laser_tf_xyz = LaunchConfiguration("laser_tf_xyz")
    laser_tf_rpy = LaunchConfiguration("laser_tf_rpy")

    use_cpp = LaunchConfiguration("use_cpp")
    launch_script_runner = LaunchConfiguration("launch_script_runner")
    launch_dobot = LaunchConfiguration("launch_dobot")
    dobot_ip = LaunchConfiguration("dobot_ip")
    dobot_type = LaunchConfiguration("dobot_type")
    overlap_roi_mode = LaunchConfiguration("overlap_roi_mode")
    overlap_roi_axis = LaunchConfiguration("overlap_roi_axis")
    overlap_roi_ratio = LaunchConfiguration("overlap_roi_ratio")

    # Vendor runtime libs for laser_driver
    ksj_lib_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "include",
        "KSJApi.bin",
        "x64",
    )

    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    merged_ld_library_path = (
        f"{ksj_lib_dir}:{existing_ld_library_path}" if existing_ld_library_path else ksj_lib_dir
    )

    disable_fastdds_shm = LaunchConfiguration("disable_fastdds_shm")

    return LaunchDescription([
        DeclareLaunchArgument("fixed_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument(
            "output_directory",
            default_value=os.path.join(os.path.expanduser("~"), "Dobot_Robot_Arm", "scans"),
        ),

        # Use C++ nodes by default (recommended).
        DeclareLaunchArgument("use_cpp", default_value="true"),
        DeclareLaunchArgument("launch_script_runner", default_value="true"),
        DeclareLaunchArgument("launch_dobot", default_value="false"),
        DeclareLaunchArgument("dobot_ip", default_value="192.168.5.1"),
        DeclareLaunchArgument("dobot_type", default_value="cr5"),
        DeclareLaunchArgument(
            "overlap_roi_mode",
            default_value="dynamic_z",
            choices=["dynamic_z", "tf_only", "fixed"],
            description="Scan accumulation mode: dynamic_z selects a Z-structured ROI for GICP, fixed uses a fixed edge ROI, tf_only disables GICP.",
        ),
        DeclareLaunchArgument("overlap_roi_axis", default_value="y"),
        DeclareLaunchArgument("overlap_roi_ratio", default_value="0.30"),

        # Workaround for FastDDS shared-memory lock errors.
        DeclareLaunchArgument("disable_fastdds_shm", default_value="true"),

        SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=merged_ld_library_path),
        SetEnvironmentVariable(
            condition=IfCondition(disable_fastdds_shm),
            name="FASTDDS_SHM_TRANSPORT_DISABLED",
            value="1",
        ),
        SetEnvironmentVariable(
            condition=IfCondition(launch_dobot),
            name="IP_address",
            value=dobot_ip,
        ),
        SetEnvironmentVariable(
            condition=IfCondition(launch_dobot),
            name="DOBOT_TYPE",
            value=dobot_type,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("cr_robot_ros2"),
                    "launch",
                    "dobot_bringup_ros2.launch.py",
                )
            ),
            condition=IfCondition(launch_dobot),
        ),

        # Optional static TF for the scanner frame.
        # Use this ONLY if your URDF is not already publishing base_link<->laser_frame.
        DeclareLaunchArgument("publish_static_laser_tf", default_value="false"),
        DeclareLaunchArgument("laser_tf_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("laser_tf_rpy", default_value="0 0 0"),

        Node(
            condition=IfCondition(publish_static_laser_tf),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_frame_static_tf",
            output="log",
            arguments=[
                # x y z roll pitch yaw frame_id child_frame_id
                laser_tf_xyz,
                laser_tf_rpy,
                "base_link",
                "laser_frame",
            ],
        ),

        # Laser scanner driver node (C++)
        Node(
            package="laser_scanner",
            executable="laser_driver",
            name="laser_driver",
            output="screen",
            emulate_tty=True,
        ),

        # -------------------- C++ pipeline --------------------
        Node(
            condition=IfCondition(use_cpp),
            package="laser_scanner",
            executable="scan_accumulator_cpp",
            name="scan_accumulator",
            output="log",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "fixed_frame": fixed_frame,
                "output_directory": output_directory,
                "overlap_roi_mode": ParameterValue(overlap_roi_mode, value_type=str),
                "overlap_roi_axis": ParameterValue(overlap_roi_axis, value_type=str),
                "overlap_roi_ratio": overlap_roi_ratio,
            }],
        ),

        Node(
            condition=IfCondition(use_cpp),
            package="laser_scanner",
            executable="map_merger_cpp",
            name="map_merger",
            output="log",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "output_directory": output_directory,
                # NOTE: C++ map_merger.cpp currently uses manual positions; it does not use odom_frame.
            }],
        ),

        Node(
            condition=IfCondition(use_cpp),
            package="laser_scanner",
            executable="scanning_coordinator_cpp",
            name="scanning_coordinator",
            output="screen",
            emulate_tty=True,
        ),

        # -------------------- Python pipeline (optional) --------------------
        Node(
            condition=UnlessCondition(use_cpp),
            package="laser_scanner",
            executable="scan_accumulator",
            name="scan_accumulator_py",
            output="log",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "fixed_frame": fixed_frame,
                "output_directory": output_directory,
                "overlap_roi_mode": ParameterValue(overlap_roi_mode, value_type=str),
                "overlap_roi_axis": ParameterValue(overlap_roi_axis, value_type=str),
                "overlap_roi_ratio": overlap_roi_ratio,
            }],
        ),

        Node(
            condition=UnlessCondition(use_cpp),
            package="laser_scanner",
            executable="map_merger",
            name="map_merger_py",
            output="log",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "output_directory": output_directory,
                "odom_frame": odom_frame,
                "robot_frame": fixed_frame,
            }],
        ),

        Node(
            condition=UnlessCondition(use_cpp),
            package="laser_scanner",
            executable="scanning_coordinator",
            name="scanning_coordinator_py",
            output="screen",
            emulate_tty=True,
        ),

        # Robot arm script runner
        Node(
            condition=IfCondition(launch_script_runner),
            package="robot_arm",
            executable="script_runner",
            name="script_runner",
            output="screen",
            emulate_tty=True,
        ),
    ])
