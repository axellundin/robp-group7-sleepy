from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    geofence_publisher_node = Node(
        package='navigation', 
        executable='geofence_publisher', 
    )

    geofence_compliance_node = Node(
        package='navigation', 
        executable='geofence_compliance', 
    )



    return LaunchDescription([
        geofence_publisher_node,
        geofence_compliance_node,
    ])

if __name__ == '__main__':
    generate_launch_description()