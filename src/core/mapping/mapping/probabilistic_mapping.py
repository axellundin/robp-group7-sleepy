#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from collections import deque
from robp_interfaces.msg import Encoders
from core_interfaces.srv import GridCreator
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformListener
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import time
import cv2 
from _thread import start_new_thread
from mapping.probabilistic_mapping_utils import ProbabilisticMapper
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

class ProbabilisticMapping(Node):
    def __init__(self):
        super().__init__('probabilistic_mapping')
        self.map_frame = "map"
        group = ReentrantCallbackGroup()
        self.grid_fill_cli = self.create_client(GridCreator, 'fill_in_workspace', callback_group=group)

        self.exploration_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_3.tsv"        
        self.resolution = 5 #cm / cell
        self.padding = 35 #cm 
        self.prob_mapper = None
        self.is_fetching_map = False 

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]  

        self.executor = None  

        self.buffer = Buffer() 
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=False)

        group2 = MutuallyExclusiveCallbackGroup()

        qos = QoSProfile(depth=1, 
                         history=HistoryPolicy.KEEP_LAST, 
                         reliability=ReliabilityPolicy.BEST_EFFORT)
         
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, callback_group=group2, qos_profile=qos)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_lidar_scan', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/local_occupancy_map', 10) 
        self.grid_pub_planning = self.create_publisher(OccupancyGrid, '/local_occupancy_map_planning', 10) 
        self.grid_pub_not_inflated = self.create_publisher(OccupancyGrid, '/local_occupancy_map_not_inflated', 10) 
        self.velocity_sub = self.create_subscription(Twist, '/current_velocity', self.velocity_callback, callback_group=group2, qos_profile=qos)

        self.curr_v = 0
        self.curr_w = 0 

        self.last_update = self.get_clock().now()
        self.update_interval = 0.5
        self.get_logger().info(f"starting local occupancy map") 
    
    def velocity_callback(self, msg):
        self.curr_v = msg.linear.x
        self.curr_w = msg.angular.z

    def fetch_map(self):
        self.is_fetching_map = True
        self.get_logger().info('fetching map')
        req = GridCreator.Request()
        req.padding = self.padding
        req.resolution = self.resolution
        req.file_name = self.exploration_workspace_file
        future = self.grid_fill_cli.call_async(req)

        while not future.done():
            time.sleep(0.01)

        flat_grid = future.result()
        self.origin = [flat_grid.grid.info.origin.position.x, flat_grid.grid.info.origin.position.y]
        grid = np.array(flat_grid.grid.data,dtype=np.float64).reshape((flat_grid.grid.info.height, flat_grid.grid.info.width))
        self.get_logger().info(f"got map") 

        self.prob_mapper = ProbabilisticMapper(grid.shape, 0.01 * self.resolution, self.origin)

    def transform_points_to_frame(self, points, transform, stamp):
        """
        transform a group of points together
        """
        transformed_points = []
        transformed_pointslist = []

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0 # point[2]
            p_stamped = PointStamped()
            p_stamped.header.stamp = stamp
            p_stamped.header.frame_id = "lidar_link" 
            p_stamped.point = p  
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
            newp = Point()
            newp.x = transformed_point_stamped.point.x
            newp.y = transformed_point_stamped.point.y
            newp.z = transformed_point_stamped.point.z
            transformed_points.append(newp)
            transformed_pointslist.append([transformed_point_stamped.point.x,transformed_point_stamped.point.y,transformed_point_stamped.point.z])
        return transformed_points, transformed_pointslist   

    def filter_points(self, scan_msg):
        """Converts a ROS2 LaserScan message into an numpy array of points."""
        # Get all valid range values and corresponding angles
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.column_stack((x, y))

        # --- Region filter parameters ---
        a = 0.085
        b = 0.12
        r = 0.39
        q = 0.31

        # Apply region mask
        x_scaled = (points[:, 0] + a) / r
        y_scaled = (points[:, 1] + b) / q
        mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) < 1
        points[mask] = 0.01

        # # Transform filtered points back to range measurements 
        # r = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        ranges[mask] = 0.01
        return ranges

    def lidar_callback(self, msg):
        if self.curr_w > 0.02 or self.curr_v > 0.05:
            self.get_logger().info("Failing due to fast movement")
            return 
        
        filtered_ranges = self.filter_points(msg)
        if filtered_ranges is None:
            return

        if self.prob_mapper is None:
           if self.is_fetching_map:
               return 
           self.fetch_map()

        self.update_grid(filtered_ranges)
        msg.header.frame_id = self.map_frame
        self.publish_filtered_scan(filtered_ranges, msg.header)

    def publish_filtered_scan(self, filtered_ranges, header):
        angles = np.arange(len(filtered_ranges)) * (np.pi / 180) - np.pi
        points = np.column_stack((filtered_ranges * np.cos(angles), filtered_ranges * np.sin(angles), np.zeros(len(filtered_ranges))))

        # Transform points to odom frame 
        try:
            transform = self.buffer.lookup_transform(self.map_frame, 'lidar_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Transform exception: {e}')
            return None

        final_points, final_pointslist = self.transform_points_to_frame(points, transform, header.stamp)
        header.frame_id = 'odom'
        msg = pc2.create_cloud(header, self.fields, final_pointslist)
        self.point_cloud_publisher.publish(msg)

    def update_grid(self, filtered_ranges): 
        try:
            transform = self.buffer.lookup_transform(self.map_frame, 'lidar_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Transform exception: {e}')
            return
        x = transform.transform.translation.x 
        y = transform.transform.translation.y  
        qx = transform.transform.rotation.x 
        qy = transform.transform.rotation.y  
        qz = transform.transform.rotation.z 
        qw = transform.transform.rotation.w 
        theta = np.arctan2(2.0*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz)
        self.prob_mapper.update_map(filtered_ranges, np.array([x,y,theta])) 

        current_time_in_seconds = self.get_clock().now().to_msg().sec 
        last_update_in_seconds = self.last_update.to_msg().sec 
        if current_time_in_seconds - last_update_in_seconds >= self.update_interval:
            self.prob_mapper.compute_inflated_occupancy()
            if self.prob_mapper is None: 
                return
            self.publish_grid_map(self.grid_pub, self.prob_mapper.inflated_map)
            self.publish_grid_map(self.grid_pub_planning, self.prob_mapper.inflated_map_planning)
            self.publish_grid_map(self.grid_pub_not_inflated, self.prob_mapper.map_binary)
            self.last_update = self.get_clock().now()

    def publish_grid_map(self, publisher, map):        
        grid = map.astype(np.int8)

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = float(self.resolution/100)
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]

        msg.info.origin.position.x = - float(self.origin[0]/100)*self.resolution
        msg.info.origin.position.y = - float(self.origin[1]/100)*self.resolution
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  
        grid = np.round(grid,0).astype(np.int8)
        grid[grid == 100] = -1
        msg.data = grid.flatten().tolist()
        publisher.publish(msg)
        # self.get_logger().info("path grid published")
    # def publish_grid_map(self, publisher, map):
    #     if self.prob_mapper is None:
    #         return
    #     # thresholded_grid = self.prob_mapper.map
    #     # thresholded_grid[thresholded_grid < -0.5] = 0 
    #     # thresholded_grid[thresholded_grid >= -0.5] = 100
        
    #     grid = self.prob_mapper.inflated_map.astype(np.int8)

    #     msg = OccupancyGrid()

    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = self.map_frame

    #     msg.info.resolution = float(self.resolution/100)
    #     msg.info.width = grid.shape[1]
    #     msg.info.height = grid.shape[0]

    #     msg.info.origin.position.x = - float(self.origin[0]/100)*self.resolution
    #     msg.info.origin.position.y = - float(self.origin[1]/100)*self.resolution
    #     msg.info.origin.position.z = 0.0
    #     msg.info.origin.orientation.w = 1.0  
    #     grid = np.round(grid,0).astype(np.int8)
    #     grid[grid == 100] = -1
    #     msg.data = grid.flatten().tolist()
        
    #     self.grid_pub.publish(msg)
    #     # self.get_logger().info("path grid published")

def main():
    rclpy.init()
    node = ProbabilisticMapping()
    ex = MultiThreadedExecutor() 
    ex.add_node(node)
    try:
        node.executor = ex 
        ex.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
