import numpy as np
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transforms 
    odom_to_baselink_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        odom_to_baselink_transform,
    ])  

if __name__ == '__main__':
    generate_launch_description()