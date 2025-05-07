from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # icp_node = Node( 
    #     package='mapping', 
    #     executable='icp_to_tf_only'
    # )

    # icp_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

    probabilistic_mapping = Node(
        package='mapping',
        executable='probabilistic_mapping'
    )

    local_occupancy = Node(
        package='mapping',
        executable='probabilistic_mapping'
    )

    # # move_to node 
    move_to_node = Node(
        package='navigation', 
        executable='move_to', 
    )

    # navigation pick_up node 
    nav_pickup = Node(
        package='navigation', 
        executable='pick_up', 
    )

    # # Odometry node 
    odometry_node = Node(
        package='odometry', 
        executable='odometry', 
    )

    path_publisher_node = Node(
        package='odometry', 
        executable='path_publisher', 
    )

    # # yolo_detection node 
    yolo_detectionnode = Node(
        package='yolo_detection', 
        executable='yolo_detector', 
    )

    # # arm_simpe_move node 
    arm_simple_move = Node(
        package='arm_control', 
        executable='simple_move', 
    )

    # # arm_box_detector node 
    arm_box_detector = Node(
        package='arm_control', 
        executable='box_detection', 
    )

    # # arm_box_pos_pub node 
    arm_box_pos_pub = Node(
        package='arm_control', 
        executable='box_position_pub', 
    )

    # # arm_place_service node 
    arm_place_service = Node(
        package='arm_control', 
        executable='place_service', 
    )

    # # move_to node 
    move_to_node = Node(
        package='navigation', 
        executable='move_to', 
    )

    # navigation pick_up node 
    nav_pickup = Node(
        package='navigation', 
        executable='pick_up', 
    )

    # # arm_pickup_service node 
    arm_picku_service = Node(
        package='arm_control', 
        executable='pickup_service', 
    )

    # # brain node 
    brain = Node(
        package='core_launch', 
        executable='brain_collection_phase', 
    )

    return LaunchDescription([
        # icp_node, 
        probabilistic_mapping,
        local_occupancy,
        odometry_node,
        path_publisher_node,
        arm_simple_move,
        arm_box_detector,
        arm_box_pos_pub,
        arm_place_service,
        move_to_node,
        nav_pickup,
        arm_picku_service
    ])  

if __name__ == '__main__':
    generate_launch_description()