#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Point
import tf2_geometry_msgs
# from tf2_ros.exceptions import TransformException #comment because error! no module called transformexceotions

class lidar_processing(Node):
    '''
    if the "lidar_launch.yaml" is launched, we will have topic "/scan"
    the node try to listen to /scan topic, read msg, transform it to point cloud 
    and publish(with time stamped and tf frame)
    '''

    def __init__(self):
        super().__init__('lidar_processing')
        print("inne i lidar")

        self.accumulated_points = []
        self.N = 10
        self.count = 0

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

        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/lidar_pointcloud',  # 话题名称
            10
        )


    def lidar_listener_callback(self, msg):
        if self.debug:
            self.get_logger().info(f"Received lidar msg, time {msg.header.stamp.sec}")
        if self.count % self.N == 0:
            self.count+=1
            return 
        self.count = 0 # walk in the process

        if self.debug:
            self.get_logger().info(f"Accept lidar msg, time {msg.header.stamp.sec}")
        
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges)# a mask to kick infinity points out
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        _time = rclpy.time.Time.from_msg(msg.header.stamp)

        # Get the transform from lidar_link to odom frame
        tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame="odom",
                source_frame="lidar_link",
                time=_time
            )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=2)
        #transforming goal pose to base_link
        try:
            t = self.tf_buffer.lookup_transform(
            "odom",
            "lidar_link",
            _time)
        except:
            self.get_logger().info("could not transform from odom to lidar link")
            return
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {"odom"} to {"lidar_link"}: {ex}')
        #     return
    

        x = ranges * np.cos(angles)# make up the points structure
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        points = np.column_stack((x, y, z))

        # Transform the points to odom frame 
        points_transformed = self.transform_points_to_frame(points, t)

        pc2_msg = pc2.create_cloud(msg.header, self.fields, points_transformed)
        pc2_msg.header.frame_id = "odom"
        
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

    def transform_points_to_frame(self, points, transform):
        """
        transform a group of points together
        """
        transformed_points = []

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            p_stamped = PointStamped()
            p_stamped.header.stamp = rclpy.time.Time()  
            p_stamped.header.frame_id = "lidar_link" 
            p_stamped.point = p  
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
            transformed_points.append([transformed_point_stamped.point.x, 
                                    transformed_point_stamped.point.y, 
                                    transformed_point_stamped.point.z])

        return np.array(transformed_points)

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