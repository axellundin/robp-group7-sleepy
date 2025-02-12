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

    arm_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('core_launch'), 
            'launch', 'launch_arm_camera.py')
        ])
    )

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
            arguments=["serial", "--dev", "/dev/hiwonder_arm", "-v6"]
        )
    
    # # Simple_move node 
    simple_move_node = Node(
        package='simple_move', 
        executable='simple_move', 
    )

    # # arm control
    joystick_arm_control = Node(
        package='arm_control', 
        executable='move_arm_hardcoded', 
    )

    # # Lidar processing
    lidar_processing_node = Node(
        package='lidar_processing', 
        executable='lidar_processing', 
    )

    # # Joystick start node 
    joy_node = Node(
        package='joy', 
        executable='joy_node', 
    )


    config_file = os.path.join(
        os.path.expanduser('~'),
        'robp-group7-sleepy/src/core/simple_move/simple_move/teleop_twist_joy.yaml'
        )
    
    # # Joystick setup node 
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file]
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
        # camera_launch,
        arm_camera_launch,
        lidar_launch,
        phidgets_launch,
        static_tf_launch,
        # odometry_node,
        joy_node, 
        teleop_node, 
        simple_move_node, 
        micro_ros_node, 
        joystick_arm_control, 
        # lidar_processing_node, 
        # camera_processing_node
    ])  

if __name__ == '__main__':
    generate_launch_description()