# src/core_launch/launch/tf_setup.launch.py
import numpy as np
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transforms 
    map_to_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    base_to_camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.08987', '0.0175', '0.10456', '0', '0', '0', 'base_link', 'camera_link']
    )

    base_to_lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0.085', '0', '0', '0', '0', 'base_link', 'lidar_link']
    )

    base_to_arm_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0.0475', '0.175', '0', '0', '0', 'base_link', 'arm_base_link']
    )

    return LaunchDescription([
        map_to_odom_transform,
        base_to_camera_transform,
        base_to_lidar_transform, 
        base_to_arm_transform
    ])