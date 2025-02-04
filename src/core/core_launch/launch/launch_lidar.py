from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="rplidar",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": "/dev/rplidar",
                "serial_baudrate": 115200,
                "frame_id": "lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": ""
            }]
        )
    ])
