import launch
import launch_ros

def generate_launch_description():
    # Launch camera launch file 
    camera_launch = launch_ros.actions.IncludeLaunchDescription(
        package='robp_launch', 
        launch='rs_d435i_launch.py',
        arguments=[]
        )  
    
    # Odometry node 
    odometry_node = launch_ros.actions.Node(
        package='core', 
        node_executable='odometry', 
        output='screen'
        )

    # Static transforms 
    static_transform_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_transform_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    return launch.LaunchDescription([
        camera_launch,
        odometry_node,
        static_transform_publisher,
        static_transform_publisher,
    ])

if __name__ == '__main__':
    launch.run(generate_launch_description())