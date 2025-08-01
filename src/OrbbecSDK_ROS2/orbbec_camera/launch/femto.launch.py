from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("depth_registration", default_value="false"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("usb_port", default_value=""),
        DeclareLaunchArgument("device_num", default_value="1"),
        DeclareLaunchArgument("product_id", default_value=""),
        DeclareLaunchArgument("enable_point_cloud", default_value="true"),
        DeclareLaunchArgument("cloud_frame_id", default_value=""),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("point_cloud_qos", default_value="default"),
        DeclareLaunchArgument("connection_delay", default_value="100"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("color_format", default_value="MJPG"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("color_flip", default_value="false"),
        DeclareLaunchArgument("color_qos", default_value="default"),
        DeclareLaunchArgument("color_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("enable_color_auto_exposure", default_value="true"),
        DeclareLaunchArgument("color_exposure", default_value="-1"),
        DeclareLaunchArgument("color_gain", default_value="-1"),
        DeclareLaunchArgument("color_brightness", default_value="-1"),
        DeclareLaunchArgument("enable_color_auto_white_balance", default_value="true"),
        DeclareLaunchArgument("color_ae_max_exposure", default_value="-1"),
        DeclareLaunchArgument("color_white_balance", default_value="-1"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="480"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("depth_format", default_value="Y16"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("depth_flip", default_value="false"),
        DeclareLaunchArgument("depth_qos", default_value="default"),
        DeclareLaunchArgument("depth_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("ir_width", default_value="640"),
        DeclareLaunchArgument("ir_height", default_value="480"),
        DeclareLaunchArgument("ir_fps", default_value="30"),
        DeclareLaunchArgument("ir_format", default_value="Y16"),
        DeclareLaunchArgument("enable_ir", default_value="true"),
        DeclareLaunchArgument("ir_flip", default_value="false"),
        DeclareLaunchArgument("ir_qos", default_value="default"),
        DeclareLaunchArgument("ir_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("enable_ir_auto_exposure", default_value="true"),
        DeclareLaunchArgument("ir_exposure", default_value="-1"),
        DeclareLaunchArgument("ir_gain", default_value="-1"),
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="true"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("accel_rate", default_value="100hz"),
        DeclareLaunchArgument("accel_range", default_value="4g"),
        DeclareLaunchArgument("enable_gyro", default_value="false"),
        DeclareLaunchArgument("gyro_rate", default_value="100hz"),
        DeclareLaunchArgument("gyro_range", default_value="1000dps"),
        DeclareLaunchArgument("liner_accel_cov", default_value="0.01"),
        DeclareLaunchArgument("angular_vel_cov", default_value="0.01"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_publish_rate", default_value="0.0"),
        DeclareLaunchArgument("ir_info_url", default_value=""),
        DeclareLaunchArgument("color_info_url", default_value=""),
        DeclareLaunchArgument("log_level", default_value="none"),
        DeclareLaunchArgument("enable_publish_extrinsic", default_value="false"),
        DeclareLaunchArgument("enable_d2c_viewer", default_value="false"),
        DeclareLaunchArgument("enable_ldp", default_value="true"),
        DeclareLaunchArgument("enable_frame_sync", default_value="false"),
        DeclareLaunchArgument("ordered_pc", default_value="false"),
        DeclareLaunchArgument("align_mode", default_value="HW"),
        DeclareLaunchArgument("laser_energy_level", default_value="-1"),
        DeclareLaunchArgument("enable_heartbeat", default_value="false"),
        DeclareLaunchArgument("time_domain", default_value="device"),
    ]

    # Node configuration
    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]
    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "foxy":
        return LaunchDescription(
            args
            + [
                Node(
                    package="orbbec_camera",
                    executable="orbbec_camera_node",
                    name="ob_camera_node",
                    namespace=LaunchConfiguration("camera_name"),
                    parameters=parameters,
                    output="screen",
                )
            ]
        )
    # Define the ComposableNode
    else:
        # Define the ComposableNode
        compose_node = ComposableNode(
            package="orbbec_camera",
            plugin="orbbec_camera::OBCameraNodeDriver",
            name=LaunchConfiguration("camera_name"),
            namespace="",
            parameters=parameters,
        )
        # Define the ComposableNodeContainer
        container = ComposableNodeContainer(
            name="camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                compose_node,
            ],
            output="screen",
        )
        # Launch description
        ld = LaunchDescription(
            args
            + [
                GroupAction(
                    [PushRosNamespace(LaunchConfiguration("camera_name")), container]
                )
            ]
        )
        return ld
