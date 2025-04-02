#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from collections import deque
from robp_interfaces.msg import Encoders
from core_interfaces.srv import GridCreator
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Point, PointStamped
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import time
import cv2 
from _thread import start_new_thread

class LocalOccupancyMap(Node):
    def __init__(self):
        super().__init__('local_occupancy_map')
        group = ReentrantCallbackGroup()
        self.grid_fill_cli = self.create_client(GridCreator, 'fill_in_workspace', callback_group=group)

        self.exploration_workspace_file = "~/robp-group7-sleepy/src/core/navigation/navigation/maps/workspace_3.tsv"        
        self.resolution = 5 #cm / cell
        self.padding = 28 #cm
        self.grid = None
        self.origin = None
        self.gaussian_kernel = None

        self.decay_factor = 0.9
        self.threshold = 30
        self.increment = 40

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]  

        self.executor = None  

        self.buffer = Buffer() 
        self.tf_listener = TransformListener(self.buffer, self, spin_thread=False)

        group2 = ReentrantCallbackGroup()

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 100, callback_group=group2)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/filtered_lidar_scan', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/local_occupancy_map', 10) 

        self.last_update = self.get_clock().now()
        self.update_interval = 0.5
        # self.create_timer(0.1, self.publish_grid_map)


    def gaussian(self, grid, sigma=2):
        max_len = max(grid.shape[0], grid.shape[1])
        gaussian_kernel = np.zeros((2 * max_len,2 * max_len)) 
        for i in range(2 * max_len):
            for j in range(2 * max_len):
                gaussian_kernel[i, j] = np.exp(-((i - max_len)**2 + (j - max_len)**2) / (2 * sigma**2))
        return gaussian_kernel 
    
    def fetch_map(self):
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

        self.gaussian_kernel = self.gaussian(grid)
        return grid
    
    def convert_to_grid_coordinates(self, x, y):
        return int(np.round((x/self.resolution)+self.origin[0],0)), int(np.round((y/self.resolution)+self.origin[1],0))

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

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

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
        mask = np.maximum(np.abs(x_scaled), np.abs(y_scaled)) >= 1
        filtered_points = points[mask]

        # KDTree-based neighbor filtering
        radius = 0.15  # 2 cm neighborhood radius
        kdtree = cKDTree(filtered_points)
        to_keep = np.ones(filtered_points.shape[0], dtype=bool)

        for i, p in enumerate(filtered_points):
            if not to_keep[i]:
                continue  # Already marked for removal
            idxs = kdtree.query_ball_point(p, r=radius)
            idxs.remove(i)  # Don't remove itself
            to_keep[idxs] = False  # Remove all close neighbors 

        # Transform points to odom frame 
        try:
            transform = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Transform exception: {e}')
            return None

        final_points = filtered_points[to_keep]
        final_points, final_pointslist = self.transform_points_to_frame(final_points, transform, scan_msg.header.stamp)
        return final_points, final_pointslist
    
    def lidar_callback(self, msg):
        result = self.filter_points(msg)
        if result is None:
            return
        _, points = result

        # Convert to ROS2 PointCloud2 message   
        msg = pc2.create_cloud(msg.header, self.fields, points)

        self.point_cloud_publisher.publish(msg) 

        if self.grid is None:
            self.grid = self.fetch_map()

        self.update_grid(points)

    def update_grid(self, points): 

        self.grid = np.round(self.grid * self.decay_factor, 0).astype(np.int8)
        
        points_to_inflate = []
        for point in points: 
            x, y = self.convert_to_grid_coordinates(point[0]* 100, point[1]* 100)
            if x < 0 or x >= self.grid.shape[1] or y < 0 or y >= self.grid.shape[0]:
                continue
            points_to_inflate.append((x, y))
            self.grid[y, x] = min(self.grid[y, x] + self.increment, 100)

        if self.get_clock().now() - self.last_update > rclpy.duration.Duration(seconds=self.update_interval):
            self.last_update = self.get_clock().now()
            start_new_thread(self.publish_grid_map, ())

    def gaussian_resample(self, grid):
        img_from_grid = grid.astype(np.uint8)
        blurred = cv2.GaussianBlur(img_from_grid, (21,21), 0)
        
        thres = 1
        temp_grid = blurred
        temp_grid[temp_grid > thres] = -1
        temp_grid[temp_grid <= thres] = 0

        return temp_grid

    def publish_grid_map(self):
        if self.grid is None:
            return
        thresholded_grid = self.grid.copy() 
        thresholded_grid[thresholded_grid < self.threshold] = 0 
        thresholded_grid[thresholded_grid >= self.threshold] = 100
        
        grid = self.gaussian_resample(thresholded_grid)

        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" 

        msg.info.resolution = float(self.resolution/100)
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]

        msg.info.origin.position.x = -float(self.origin[0]/100)*self.resolution
        msg.info.origin.position.y = -float(self.origin[1]/100)*self.resolution
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  
        grid = np.round(grid,0).astype(np.int8)
        msg.data = grid.flatten().tolist()
        
        self.grid_pub.publish(msg)
        self.get_logger().info("path grid published")

def main():
    rclpy.init()
    node = LocalOccupancyMap()
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
