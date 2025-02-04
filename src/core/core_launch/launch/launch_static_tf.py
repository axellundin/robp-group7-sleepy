# src/core_launch/launch/tf_setup.launch.py

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
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    base_to_lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']
    )

    return LaunchDescription([
        map_to_odom_transform,
        base_to_camera_transform,
        base_to_lidar_transform
    ])