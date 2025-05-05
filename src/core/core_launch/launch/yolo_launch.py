from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # # Odometry node 
    camera_processing_node = Node(
        package='perception', 
        executable='camera_processing', 
    )

    map_generator_node = Node(
        package='perception', 
        executable='mapg', 
    )

    yolo_detection_node = Node(
        package='yolo_detection', 
        executable='yolo_detector', 
    )

    return LaunchDescription([
        camera_processing_node,
        map_generator_node,
        yolo_detection_node,
    ])  

if __name__ == '__main__':
    generate_launch_description()