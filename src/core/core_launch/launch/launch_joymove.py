from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch motor launch file 
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robp_launch'), 
            'launch', 'phidgets_launch.py')
        ])
    )
    
    
    # # Simple_move node 
    simple_move_node = Node(
        package='simple_move', 
        executable='simple_move', 
    )

    # # Jaystick start node 
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

    return LaunchDescription([
        joy_node,
        teleop_node,
        motor_launch,
        simple_move_node,
    ])

if __name__ == '__main__':
    generate_launch_description()