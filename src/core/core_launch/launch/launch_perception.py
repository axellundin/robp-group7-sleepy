from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch camera launch file 
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robp_launch'), 
            'launch', 'rs_d435i_launch.py')
        ])
    )

    phidgets_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robp_launch'), 
            'launch', 'phidgets_launch.py')
        ])
    )


    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('core_launch'), 
            'launch', 'launch_static_tf.py')
        ])
    )
    
    # # Odometry node 
    odometry_node = Node(
        package='odometry', 
        executable='odometry', 
    )
    
    camera_processing_node = Node(
        package='perception', 
        executable='camera_processing', 
    )

    return LaunchDescription([
        phidgets_launch,
        static_tf_launch,
        odometry_node,
        camera_launch,
        camera_processing_node
    ])

if __name__ == '__main__':
    generate_launch_description()