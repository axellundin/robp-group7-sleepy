from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # # move_to node 
    move_to_node = Node(
        package='navigation', 
        executable='move_to', 
        arguments=['--ros-args', '--log-level', 'move_to:=DEBUG']
    )

    # navigation pick_up node 
    nav_pickup = Node(
        package='navigation', 
        executable='pick_up', 
        arguments=['--ros-args', '--log-level', 'PickUp:=DEBUG']
    )

    return LaunchDescription([
        move_to_node,
        nav_pickup,
    ])  

if __name__ == '__main__':
    generate_launch_description()