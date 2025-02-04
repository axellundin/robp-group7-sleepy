#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class lidar_processing(Node):
    '''
    if the "lidar_launch.yaml" is launched, we will have topic "/scan"
    the node try to listen to /scan topic, read msg, transform it to point cloud 
    and publish(with time stamped and tf frame)
    '''

    def __init__(self):
        super().__init__('lidar_processing')

        self.accumulated_points = []

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]  

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value
        if self.debug:
            self.get_logger().info('Debug mode enabled')
            self.get_logger().info('node lidar_processing inited')

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_listener_callback,
            10
        )
        self.subscription

        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/lidar_pointcloud',  # 话题名称
            10
        )


    def lidar_listener_callback(self, msg):
        if self.debug:
            self.get_logger().info(f"Received lidar msg, time {msg.header.stamp.sec}")
        
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        valid_indices = np.isfinite(ranges)# a mask to kick infinity points out
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        x = ranges * np.cos(angles)# make up the points structure
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        points = np.column_stack((x, y, z))
        
        self.accumulated_points.extend(points)

      
        pc2_msg = pc2.create_cloud(msg.header, self.fields, self.accumulated_points)
        
        self.point_cloud_publisher.publish(pc2_msg)

        '''
        a classic lidar msg has structure below:
        header:
            stamp:
                sec: 1738594286
                nanosec: 567656115
            frame_id: lidar_link
        angle_min: -3.1415927410125732
        angle_max: 3.1415927410125732
        angle_increment: 0.017501909285783768
        time_increment: 0.0003824508748948574
        scan_time: 0.13729986548423767
        range_min: 0.05000000074505806
        range_max: 16.0
        ranges:
        - 0.4000000059604645
        - 0.4009999930858612
        - .inf
        - 0.4000000059604645
        - 0.4000000059604645
        - '...'
        intensities:
        - 47.0
        - 47.0
        - 0.0
        - 47.0
        - 47.0
        - '...'
        '''

def main():
    rclpy.init()
    node = lidar_processing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()