#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
