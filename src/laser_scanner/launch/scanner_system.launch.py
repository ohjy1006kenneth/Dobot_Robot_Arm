#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF for robot_state_publisher (provides TF tree for arm FK)
    urdf_path = os.path.join(
        os.path.expanduser('~'),
        'Dobot_Robot_Arm', 'src', 'surface_reconstruction', 'urdf',
        'surface_reconstruction.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot state publisher — broadcasts TF from joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/joint_states', '/joint_states_robot')],
        ),

        # Laser scanner driver node
        Node(
            package='laser_scanner',
            executable='laser_driver',
            name='laser_driver',
            output='screen',
            emulate_tty=True,
        ),
        
        # Scan accumulator node
        Node(
            package='laser_scanner',
            executable='scan_accumulator',
            name='scan_accumulator',
            output='screen',
            emulate_tty=True,
        ),
        
        # Scanning coordinator node
        Node(
            package='laser_scanner',
            executable='scanning_coordinator',
            name='scanning_coordinator',
            output='screen',
            emulate_tty=True,
        ),
    ])
