from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    static_map_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('core_launch'), 
            'launch', 'launch_static_tf_map_odom.py')
        ])
    )

    return LaunchDescription([
        static_map_odom
       
    ])  

if __name__ == '__main__':
    generate_launch_description()