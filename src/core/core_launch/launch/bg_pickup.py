from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('core_launch'), 
            'launch', 'launch_lidar.py')
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

    micro_ros_node = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/hiwonder_arm", "-v0"]
        )

    # # Simple_move node 
    simple_move_node = Node(
        package='simple_move', 
        executable='simple_move', 
    )


    # # grid creator node 
    grid_creator = Node(
        package='navigation', 
        executable='grid_map_creator', 
    )


    # # yolo_detection node 
    yolo_detectionnode = Node(
        package='yolo_detection', 
        executable='yolo_detector', 
    )

    # # arm_joint_pub node 
    arm_joint_pub = Node(
        package='arm_control', 
        executable='arm_joint_tf_pub', 
    )

    # Tick accumulator node 
    tick_accumulator = Node(
        package='odometry', 
        executable='tick_accumulator', 
    )

    return LaunchDescription([
        lidar_launch,
        phidgets_launch,
        simple_move_node,
        micro_ros_node,
        grid_creator,
        static_tf_launch,
        # yolo_detectionnode,
        arm_joint_pub,
        tick_accumulator,
    ])  

if __name__ == '__main__':
    generate_launch_description()