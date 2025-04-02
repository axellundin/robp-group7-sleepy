import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mapping',
            executable='mapping',
            name='icp_mapping_node'
        ),
        launch_ros.actions.Node(
            package='your_package',
            executable='corrected_path_publisher',
            name='corrected_path_publisher'
        ),
    ])
